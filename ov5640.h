/* SPDX-License-Identifier: GPL-2.0+ */
#ifndef OV5640_H
#define OV5640_H

#define OV5640_CHIP_ID_HIGH_BYTE        0x300A
#define OV5640_CHIP_ID_LOW_BYTE         0x300B
#define OV5640_SYSTEM_RESET00           0x3000
#define OV5640_CLOCK_ENABLE00           0x3004
#define OV5640_OTP_PROGRAM_CTRL         0x3D20
#define OV5640_OTP_READ_CTRL            0x3D21

#define OV5640_OTP_START_ADDR           0x3D05
#define OV5640_OTP_END_ADDR             0x3D1F
#define OV5640_SENSOR_MODEL_ID_ADDR     0x3D06

#define OV5640_SENSOR_MODEL_MAX_LEN     22
#define OV5640_SENSOR_MODEL_HIGH_K      "OV5640-A71A-K_45039C15"
#define OV5640_SENSOR_MODEL_CSP         "OV5640-A71A_45039C15J"
#define OV5640_SENSOR_MODEL_HIGH_K_ID   0x02

struct reg_value {
	u16 u16RegAddr;
	u8 u8Val;
};
int ov5640_doi2cwrite(struct device *dev, struct reg_value *pMode, USHORT elements);
DWORD ov5640_flipimage(struct device *dev, bool flip);
void ov5640_enable_stream(struct device *dev, bool enable);
int ov5640_create_sysfs_attributes(struct device *dev);
void ov5640_remove_sysfs_attributes(struct device *dev);
int ov5640_init(struct device *dev);
int ov5640_ioctl(struct device *dev, int cmd, PUCHAR pBuf, PUCHAR pUserBuf);

#endif
