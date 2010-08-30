/*
 * Shuttle VFD (20x1 character display. Each character cell is 5x8 pixels)
 * - The display is driven by Princeton Technologies PT6314 VFD controller
 * - Cypress CY7C63723C (receives USB commands and control VFD controller)
 *
 * Tested on Shuttle XPC models: SG33G5M.
 *
 * Copyright (C) 2009-2010 Matthieu Crapet <mcrapet@gmail.com>
 * Based on some "drivers/usb/misc" sources
 *
 * LCD "prococol" : each message has a length of 8 bytes
 * - 1 nibble: command (0x1, 0x3, 0x7, 0x9, 0xD)
 *     - 0x1 : clear text and icons (len=1)
 *     - 0x7 : icons (len=4)
 *     - 0x9 : text (len=7)
 *     - 0xD : set clock data (len=7)
 *     - 0x3 : display clock (internal feature) (len=1)
 * - 1 nibble: message length (0-7)
 * - 7 bytes : message data
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/usb.h>
#include <linux/version.h>
#include <linux/kernel_stat.h>
#include <linux/slab.h>
#include <linux/leds.h>

#define SHUTTLE_VFD_VENDOR_ID           0x051C

/* VFD physical dimensions */
#define SHUTTLE_VFD_WIDTH               20
#define SHUTTLE_VFD_HEIGHT              1  // not used

/* VFD USB control message */
#define SHUTTLE_VFD_PACKET_SIZE         8
#define SHUTTLE_VFD_DATA_SIZE           (SHUTTLE_VFD_PACKET_SIZE-1)
#define SHUTTLE_VFD_SLEEP_MS            24
#define SHUTTLE_VFD_INTERFACE           1

/* VFS Icons (volume icon is not included) */
#define NUM_LEDS 15
#define SHUTTLE_VFD_ICON_VOL(x)         ((x) << 15)
#define SHUTTLE_VFD_ICON_VOL_NUM        12
#define SHUTTLE_VFD_BASE_MASK           0x7FFF


/* Table of devices that work with this driver */
static struct usb_device_id shuttle_vfd_table [] = {
	{ USB_DEVICE(SHUTTLE_VFD_VENDOR_ID, 0x0003) },
	{ USB_DEVICE(SHUTTLE_VFD_VENDOR_ID, 0x0005) },
	{ }
};
MODULE_DEVICE_TABLE(usb, shuttle_vfd_table);

static const char *icon_names[NUM_LEDS] = {
	"tv", "cd", "music", "radio", "clock", "pause", "play", "record",
	"rewind", "camera", "mute", "repeat", "reverse", "fastforward", "stop"
};

/* Working structure */
struct vfd_led {
	unsigned long mask;
	char name[16];
	struct led_classdev led_cdev;
	struct shuttle_vfd *vfd;
};

struct shuttle_vfd {
	struct usb_device *udev;
	struct mutex vfd_mutex;

	unsigned long icons_mask; /* global mask */
	unsigned char packet[SHUTTLE_VFD_PACKET_SIZE];
	unsigned char screen[SHUTTLE_VFD_WIDTH];

	struct vfd_led icons[NUM_LEDS];
	struct vfd_led volume;
};

/* Local prototypes */
static int vfd_send_packet(struct shuttle_vfd *, unsigned char *);
static inline void vfd_reset_cursor(struct shuttle_vfd *, bool);
static inline void vfd_set_icons(struct shuttle_vfd *);
static inline void vfd_set_text(struct shuttle_vfd *, size_t);


static int vfd_send_packet(struct shuttle_vfd *vfd, unsigned char *packet)
{
	int result;

	mutex_lock(&vfd->vfd_mutex);
	result = usb_control_msg(vfd->udev,
			usb_sndctrlpipe(vfd->udev, 0),
			0x09,
			0x21,    // HID class
			0x0200,
			SHUTTLE_VFD_INTERFACE,
			(char *) (packet) ? packet : vfd->packet,
			SHUTTLE_VFD_PACKET_SIZE,
			USB_CTRL_GET_TIMEOUT / 4);

	/* this sleep inside the critical section is not very nice,
	 * but it avoids screw-up display on conccurent access */
	msleep(SHUTTLE_VFD_SLEEP_MS);

	mutex_unlock(&vfd->vfd_mutex);
	if (result < 0)
		dev_err(&vfd->udev->dev, "send packed failed: %d\n", result);

	return result;
}

static inline void vfd_reset_cursor(struct shuttle_vfd *vfd, bool eraseall)
{
	memset(&vfd->packet[0], 0, SHUTTLE_VFD_PACKET_SIZE);
	vfd->packet[0] = (1 << 4) + 1;

	if (eraseall)
		vfd->packet[1] = 1; // full clear (text + icons)
	else
		vfd->packet[1] = 2; // just reset the text cursor (keep text)

	vfd_send_packet(vfd, NULL);
}

static inline void vfd_set_icons(struct shuttle_vfd *vfd)
{
	memset(&vfd->packet[0], 0, SHUTTLE_VFD_PACKET_SIZE);
	vfd->packet[0] = (7 << 4) + 4;
	vfd->packet[1] = (vfd->icons_mask >> 15) & 0x1F;
	vfd->packet[2] = (vfd->icons_mask >> 10) & 0x1F;
	vfd->packet[3] = (vfd->icons_mask >>  5) & 0x1F;
	vfd->packet[4] = vfd->icons_mask & 0x1F; // each data byte is stored on 5 bits
	vfd_send_packet(vfd, NULL);
}

static inline void vfd_set_text(struct shuttle_vfd *vfd, size_t len)
{
	size_t i;
	char *p = (char *)&vfd->screen[0];

	for (i = 0; i < (len/SHUTTLE_VFD_DATA_SIZE); i++) {
		vfd->packet[0] = (9 << 4) + SHUTTLE_VFD_DATA_SIZE;
		memcpy(vfd->packet + 1, p, SHUTTLE_VFD_DATA_SIZE);
		p += SHUTTLE_VFD_DATA_SIZE;
		vfd_send_packet(vfd, NULL);
	}

	len = len % SHUTTLE_VFD_DATA_SIZE;
	if (len != 0) {
		memset(&vfd->packet[0], 0, SHUTTLE_VFD_PACKET_SIZE);
		vfd->packet[0] = (9 << 4) + len;
		memcpy(vfd->packet + 1, p, len);
		vfd_send_packet(vfd, NULL);
	}
}

/* attribute callback handler (text write) */
static ssize_t set_vfd_text_handler(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct usb_interface *intf = to_usb_interface(dev);
	struct shuttle_vfd *vfd = usb_get_intfdata(intf);
	size_t l;

	if (count < SHUTTLE_VFD_WIDTH)
		memset(&vfd->screen[0], 0, SHUTTLE_VFD_WIDTH);

	l = min(count, (size_t)SHUTTLE_VFD_WIDTH);
	memcpy(&vfd->screen[0], buf, l);
	vfd_reset_cursor(vfd, false);
	vfd_set_text(vfd, SHUTTLE_VFD_WIDTH);

	return count;
}

/* attribute callback handler (text read) */
static ssize_t get_vfd_text_handler(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t sz = SHUTTLE_VFD_WIDTH;

	struct usb_interface *intf = to_usb_interface(dev);
	struct shuttle_vfd *vfd = usb_get_intfdata(intf);

	memcpy(buf, &vfd->screen[0], SHUTTLE_VFD_WIDTH);

	while (buf[sz-1] == '\0' || buf[sz-1] == '\n')
		sz--;
	buf[sz] = '\n';

	return sz + 1;
}

static void vfd_led_generic_set(struct led_classdev *led_cdev,
		enum led_brightness value)
{
	struct vfd_led *led = container_of(led_cdev, struct vfd_led, led_cdev);

	if (value != LED_OFF)
		led->vfd->icons_mask |= led->mask;
	else
		led->vfd->icons_mask &= ~led->mask;

	vfd_set_icons(led->vfd);
}

static void vfd_led_volume_set(struct led_classdev *led_cdev,
		enum led_brightness value)
{
	struct vfd_led *led = container_of(led_cdev, struct vfd_led, led_cdev);

	led->vfd->icons_mask &= SHUTTLE_VFD_BASE_MASK;
	if (value != LED_OFF)
		led->vfd->icons_mask |= SHUTTLE_VFD_ICON_VOL(value);
	vfd_set_icons(led->vfd);
}

static DEVICE_ATTR(text, S_IWUGO | S_IRUGO,
		get_vfd_text_handler, set_vfd_text_handler);

static int shuttle_vfd_probe(struct usb_interface *interface,
		const struct usb_device_id *id)
{
	struct vfd_led *led;
	struct led_classdev *led_cdev;

	struct shuttle_vfd *dev = NULL;
	int i, retval = -ENOMEM;

	if (interface->cur_altsetting->desc.bInterfaceNumber !=
			SHUTTLE_VFD_INTERFACE)
		goto error_mem;

	dev = kzalloc(sizeof(struct shuttle_vfd), GFP_KERNEL);
	if (!dev) {
		dev_err(&interface->dev, "Out of memory\n");
		goto error_mem;
	}

	dev->udev = usb_get_dev(interface_to_usbdev(interface));
	dev->icons_mask = 0;

	usb_set_intfdata(interface, dev);
	mutex_init(&dev->vfd_mutex);

	for (i = 0; i < NUM_LEDS; i++) {
		led = &dev->icons[i];
		snprintf(led->name, sizeof(led->name), icon_names[i]);
		led->vfd = dev;
		led->mask = 1 << i;

		led_cdev = &led->led_cdev;
		led_cdev->name = led->name;
		led_cdev->max_brightness = 1; /* on/off */
		led_cdev->brightness = LED_OFF;
		led_cdev->brightness_set = vfd_led_generic_set;

		retval = led_classdev_register(&dev->udev->dev, led_cdev);
		if (retval < 0)
			goto error;
	}

	led = &dev->volume;
	snprintf(led->name, sizeof(led->name), "volume");
	led->vfd = dev;
	led->mask = 0;

	led_cdev = &led->led_cdev;
	led_cdev->name = led->name;
	led_cdev->max_brightness = SHUTTLE_VFD_ICON_VOL_NUM;
	led_cdev->brightness = LED_OFF;
	led_cdev->brightness_set = vfd_led_volume_set;

	retval = led_classdev_register(&dev->udev->dev, led_cdev);
	if (retval < 0)
		goto error;

	/* create device attribute file */
	retval = device_create_file(&interface->dev, &dev_attr_text);
	if (retval)
		goto error2;

	vfd_reset_cursor(dev, true);

	return 0;

error2:
	led_classdev_unregister(&dev->volume.led_cdev);
error:
	for (i--; i >= 0; i--)
		led_classdev_unregister(&dev->icons[i].led_cdev);
	usb_set_intfdata (interface, NULL);
	usb_put_dev(dev->udev);
	kfree(dev);

error_mem:
	return retval;
}

static void shuttle_vfd_disconnect(struct usb_interface *interface)
{
	struct shuttle_vfd *dev;
	int i;

	dev = usb_get_intfdata(interface);

	for (i = 0; i < NUM_LEDS; i++)
		led_classdev_unregister(&dev->icons[i].led_cdev);
	led_classdev_unregister(&dev->volume.led_cdev);

	device_remove_file(&interface->dev, &dev_attr_text);

	/* the intfdata can be set to NULL only after the
	 * device files have been removed */
	usb_set_intfdata(interface, NULL);
	usb_put_dev(dev->udev);

	kfree(dev);
}

static struct usb_driver shuttle_vfd_driver = {
	.name		= "shuttle_leds",
	.probe		= shuttle_vfd_probe,
	.disconnect	= shuttle_vfd_disconnect,
	.id_table	= shuttle_vfd_table
};

static int __init shuttle_vfd_init(void)
{
	int err;

	err = usb_register(&shuttle_vfd_driver);
	if (err) {
		err("Function usb_register failed! Error number: %d\n", err);
	}

	return err;
}

static void __exit shuttle_vfd_exit(void)
{
	usb_deregister(&shuttle_vfd_driver);
}

module_init(shuttle_vfd_init);
module_exit(shuttle_vfd_exit);

MODULE_DESCRIPTION("Shuttle VFD LEDs driver");
MODULE_VERSION("1.00");
MODULE_AUTHOR("Matthieu Crapet <mcrapet@gmail.com>");
MODULE_LICENSE("GPL");
