#ifndef	__HPET__
#define	__HPET__ 1



struct hpet_info {
	unsigned long hi_ireqfreq;	/* Hz */
	unsigned long hi_flags;	/* information */
	unsigned short hi_hpet;
	unsigned short hi_timer;
};

#define	HPET_INFO_PERIODIC	0x0001	/* timer is periodic */

#define	HPET_IE_ON	_IO('h', 0x01)	/* interrupt on */
#define	HPET_IE_OFF	_IO('h', 0x02)	/* interrupt off */
#define	HPET_INFO	_IOR('h', 0x03, struct hpet_info)
#define	HPET_EPI	_IO('h', 0x04)	/* enable periodic */
#define	HPET_DPI	_IO('h', 0x05)	/* disable periodic */
#define	HPET_IRQFREQ	_IOW('h', 0x6, unsigned long)	/* IRQFREQ usec */

#endif				/* !__HPET__ */
