// SPDX-License-Identifier: GPL-2.0-only
/*
 * Prophesee FPGA CSI Rx driver
 *
 * Copyright (C) Prophesee S.A.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>

#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>

#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>

#define OUT 0
#define IN 1
#define NB_DMA_CHAN 2

#define BYTES_PER_LINE (1lu << 20)
#define SIZE_IMAGE BYTES_PER_LINE

// Register map
// SYSTEM CONFIG
#define SYSTEM_ID 0x00000800
#define BITSTREAM_VERSION 0x00000804
#define BUILD_DATE 0x00000808
#define VERSION_CONTROL_ID 0x0000080C

// SYSTEM CONTROL
#define GLOBAL_CONTROL 0x00000000
#define CLK_CONTROL 0x00000004
#define TIME_BASE_CONTROL 0x00000008
#define TIME_BASE_RESOLUTION 0x0000000C
#define IMU_CONTROL 0x00000010
#define EVT_DATA_FORMATTER_CONTROL 0x00000014
#define EVT_MERGE_CONTROL 0x00000018
#define TH_RECOVERY_CONTROL 0x00000044
#define TS_CHECKER_CONTROL 0x00000048
#define TS_CHECKER_EVT_UI_CNT 0x0000004C
#define TS_CHECKER_EVT_BROKEN_CNT 0x00000050
#define TS_CHECKER_EVT_UK_ERR_CNT 0x00000054
#define BOARD_CONTROL_STATUS 0x00000058
#define IO_CONTROL 0x0000005C
#define OUT_TH_RECOVERY_CONTROL 0x00000060
#define OUT_TS_CHECKER_CONTROL 0x00000064
#define OUT_TS_CHECKER_EVT_UI_CNT 0x00000068
#define OUT_TS_CHECKER_EVT_BROKEN_CNT 0x0000006C
#define OUT_TS_CHECKER_EVT_UK_ERR_CNT 0x00000070
#define IP_SL_MXTA_MONITOR_CONTROL 0x00000074

// HUAHINE IF CTRL
#define IF_CONTROL 0x0070F000
#define IF_TRIGGER 0x0070F004
#define IF_TEST_PATTERN_CONTROL 0x0070F008
#define IF_TEST_PATTERN_N_PERIOD 0x0070F00C
#define IF_TEST_PATTERN_P_PERIOD 0x0070F010
#define IF_TEST_PATTERN_VECTOR 0x0070F014
#define IF_OOB_FILTER_CONTROL 0x0070F018
#define IF_OOB_FILTER_ORIGIN 0x0070F01C
#define IF_OOB_FILTER_SIZE 0x0070F020
#define IF_CCAM5_CONTROL 0x0070F024
#define IF_TRIGGER_FWD 0x0070F028
#define IF_MIPI_RX_CONTROL 0x0070F02C
#define IF_HUAHINE_CTRL 0x0070F030
#define IF_HAHINE_POWER_CTRL 0x0070F034
#define IF_HUAHINE_CLK_CTRL 0x0070F038

// MIPI RX IP
#define MIPI_CORE_CONFIG 0X00700000

// AUX IF 
// IGNORING

// VERIFICATION IP (VIP)
#define VIP_CONTROL 0x00710100
#define VIP_TEST_STATUS 0x00710104
#define VIP_PATTERN_TEST_DATA 0x00710108
#define VIP_TEST_RESULT 0x0071010C
#define VIP_VALID_COUNTER 0x00710110
#define VIP_ERROR_COUNTER 0x00710114
#define VIP_ERROR_INDEX 0x00710118

// SYSTEM MONITOR
// IGNORING MOST
#define MONITOR_CONTROL_EVT_MERGE_CONTROL 0x00000600

// PS HOST IF
#define PS_AXI_DMA_PACKETIZER_CONTROL 0x00002000
#define PS_AXI_DMA_PACKETIZER_PACKET_LENGTH 0x00002004
#define PS_AXIL_BRIDGE_CONTROL 0x00002100
#define PS_AXIL_BRIDGE_ERROR_STATUS 0x00002104

// IMX636 MIPI
#define IMX636_MIPI_BASE 		  0x00100000
#define IMX636_GLOBAL_CTRL        (IMX636_MIPI_BASE + 0x00000000)
#define IMX636_ROI_CTRL           (IMX636_MIPI_BASE + 0x00000004)
#define IMX636_CHIP_ID            (IMX636_MIPI_BASE + 0x00000014)
#define IMX636_SPARE_CTRL_0       (IMX636_MIPI_BASE + 0x00000018)
#define IMX636_DIVISOR_CTRL       (IMX636_MIPI_BASE + 0x000000B8)
#define IMX636_GLOBAL_CTRL_2      (IMX636_MIPI_BASE + 0x000000C0)
#define IMX636_EDF_PIPE_CTRL      (IMX636_MIPI_BASE + 0x00007000)
#define IMX636_EOI_PIPE_CTRL      (IMX636_MIPI_BASE + 0x00008000)
#define IMX636_RO_CTRL            (IMX636_MIPI_BASE + 0x00009000)
#define IMX636_RO_TIME_BASE_CTRL  (IMX636_MIPI_BASE + 0x00009008)
#define IMX636_RO_LOOP_CTRL       (IMX636_MIPI_BASE + 0x00009028)
#define IMX636_MIPI_CTRL          (IMX636_MIPI_BASE + 0x0000B000)
#define IMX636_MIPI_ESC_CTRL      (IMX636_MIPI_BASE + 0x0000B004)
#define IMX636_MIPI_POWER         (IMX636_MIPI_BASE + 0x0000B040)
#define IMX636_MIPI_STREAM        (IMX636_MIPI_BASE + 0x0000B044)
#define IMX636_MIPI_PL_RG_1       (IMX636_MIPI_BASE + 0x0000B064)
#define IMX636_MIPI_PL_RG_2       (IMX636_MIPI_BASE + 0x0000B068)
#define IMX636_MIPI_PL_RG_5       (IMX636_MIPI_BASE + 0x0000B074)
#define IMX636_MIPI_PL_RG_6       (IMX636_MIPI_BASE + 0x0000B078)
#define IMX636_MIPI_PL_RG_7       (IMX636_MIPI_BASE + 0x0000B07C)
#define IMX636_MIPI_TCLKPOST      (IMX636_MIPI_BASE + 0x0000B080)
#define IMX636_MIPI_TCLKPRE       (IMX636_MIPI_BASE + 0x0000B084)
#define IMX636_MIPI_TCLKPREPARE   (IMX636_MIPI_BASE + 0x0000B088)
#define IMX636_MIPI_TCLKTRAIL     (IMX636_MIPI_BASE + 0x0000B08C)
#define IMX636_MIPI_TCLKZERO      (IMX636_MIPI_BASE + 0x0000B090)
#define IMX636_MIPI_THSEXIT       (IMX636_MIPI_BASE + 0x0000B094)
#define IMX636_MIPI_THSPREPARE    (IMX636_MIPI_BASE + 0x0000B098)
#define IMX636_MIPI_THSZERO       (IMX636_MIPI_BASE + 0x0000B09C)
#define IMX636_MIPI_THSTRAIL      (IMX636_MIPI_BASE + 0x0000B0A0)
#define IMX636_MIPI_TLPX          (IMX636_MIPI_BASE + 0x0000B0A4)
#define IMX636_MIPI_TXCLKESC_FREQ (IMX636_MIPI_BASE + 0x0000B0AC)
#define IMX636_MIPI_DPHY_POWER    (IMX636_MIPI_BASE + 0x0000B0C8)
#define IMX636_MIPI_DPHY_PLL_DIV  (IMX636_MIPI_BASE + 0x0000B0CC)
#define IMX636_MIPI_BYTECLK_CTRL  (IMX636_MIPI_BASE + 0x0000B120)
#define IMX636_SLVS_LINKCLK_CTRL  (IMX636_MIPI_BASE + 0x0000E120)
#define IMX636_MBX_MISC           (IMX636_MIPI_BASE + 0x00400010)

// MIPI Timings - 600 Mbps
#define PLL_FB_DIV_D_600   120
#define DVTOP_DIV_D_600   0x08
#define SYS_CLK_DIV_D_600    2
#define TCLKPOST_600       103
#define TCLKPRE_600         15
#define TCLKPREPARE_600     39
#define TCLKTRAIL_600       39
#define TCLKZERO_600       183
#define THSEXIT_600         71
#define THSPREPARE_600      47
#define THSZERO_600         79
#define THSTRAIL_600        47
#define TLPX_600            39
#define TXCLKESC_FREQ_600   40
#define DPHY_CLK_DIV_600     1

static int video_nr = -1;
module_param(video_nr, uint, 0644);
MODULE_PARM_DESC(video_nr, "videoX start number, -1 is autodetect");

struct psee_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
	dma_cookie_t dma_cookie;
};

static inline struct psee_buffer *to_psee_buffer(struct vb2_buffer *vb2)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb2);

	return container_of(vbuf, struct psee_buffer, vb);
}

struct psee_video {
	struct media_device mdev;
	struct video_device vdev;
	struct v4l2_device v4l2_dev;
	struct dma_chan *chan[NB_DMA_CHAN];	/* dma support */
	struct mutex lock;
	struct vb2_queue queue;
	spinlock_t qlock;
	struct list_head buffers;
	int sequence;
	struct resource *reg_resource;
	void __iomem *regmap;
};

static inline u32 read_reg(struct psee_video *p, u32 reg)
{
	return readl(((char *)p->regmap) + reg);
}

static inline void write_reg(struct psee_video *p, u32 reg, u32 value)
{
	writel(value, ((char *)p->regmap) + reg);
	usleep_range(1000, 2000);
}

static int psee_video_try_format(struct psee_video *pdata, u32 which,
			   struct v4l2_pix_format *pix_fmt,
			   struct v4l2_rect *crop, struct v4l2_rect *compose)
{
	pix_fmt->width = 1280;
	pix_fmt->height = 720;
	pix_fmt->field = V4L2_FIELD_NONE;
	pix_fmt->colorspace = V4L2_COLORSPACE_RAW;
	pix_fmt->pixelformat = v4l2_fourcc('P', 'S', 'E', 'E');
	pix_fmt->flags = V4L2_FMT_FLAG_COMPRESSED;
	pix_fmt->xfer_func = V4L2_XFER_FUNC_NONE;
	pix_fmt->bytesperline = BYTES_PER_LINE;
	pix_fmt->sizeimage = SIZE_IMAGE;

	if (crop) {
		crop->top = 0;
		crop->left = 0;
		crop->width = pix_fmt->width;
		crop->height = pix_fmt->height;
	}

	if (compose) {
		compose->top = 0;
		compose->left = 0;
		compose->width = pix_fmt->width;
		compose->height = pix_fmt->height;
	}

	return 0;
}

static int psee_s_fmt_vid_cap(struct file *file, void *priv,
			      struct v4l2_format *f)
{
	struct psee_video *pdata = video_drvdata(file);
	struct v4l2_rect crop, compose;

	if (vb2_is_busy(&pdata->queue))
		return -EBUSY;

	return psee_video_try_format(pdata, V4L2_SUBDEV_FORMAT_ACTIVE, &f->fmt.pix,
			      &crop, &compose);
}

static int psee_g_fmt_vid_cap(struct file *file, void *priv,
			      struct v4l2_format *f)
{
	f->fmt.pix.width = 1280;
	f->fmt.pix.height = 720;
	f->fmt.pix.field = V4L2_FIELD_NONE;
	f->fmt.pix.colorspace = V4L2_COLORSPACE_RAW;
	f->fmt.pix.pixelformat = v4l2_fourcc('P', 'S', 'E', 'E');
	f->fmt.pix.flags = V4L2_FMT_FLAG_COMPRESSED;
	f->fmt.pix.xfer_func = V4L2_XFER_FUNC_NONE;
	f->fmt.pix.bytesperline = BYTES_PER_LINE;
	f->fmt.pix.sizeimage = SIZE_IMAGE;

	return 0;
}

static int psee_enum_fmt_vid_cap(struct file *file, void *priv,
				 struct v4l2_fmtdesc *f)
{
	if (f->index != 0)
		return -EINVAL;
	f->pixelformat = v4l2_fourcc('P', 'S', 'E', 'E');
	f->flags = V4L2_FMT_FLAG_COMPRESSED;
	strscpy(f->description, "Prophesee EVT3.0", sizeof(f->description));

	return 0;
}

static int psee_try_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct psee_video *pdata = video_drvdata(file);

	return psee_video_try_format(pdata, V4L2_SUBDEV_FORMAT_TRY, &f->fmt.pix, NULL, NULL);
}

static int psee_video_initialize_device(struct file *file)
{
	struct psee_video *pdata = video_drvdata(file);
	int ret;

	struct v4l2_format f = {
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
		.fmt.pix = {
			.width		= 1280,
			.height		= 720,
			.field		= V4L2_FIELD_NONE,
			.colorspace	= V4L2_COLORSPACE_RAW,
			.pixelformat	= v4l2_fourcc('P', 'S', 'E', 'E'),
			.bytesperline	= BYTES_PER_LINE,
			.sizeimage	= SIZE_IMAGE,
		},
	};

	/* Just apply evk2_imx636_cd sequence */
	write_reg(pdata, CLK_CONTROL, 0x00000111); // CLK_CONTROL
	write_reg(pdata, GLOBAL_CONTROL, 0x00000089); // GLOBAL_CONTROL
	write_reg(pdata, GLOBAL_CONTROL, 0x0000008D); // GLOBAL_CONTROL
	write_reg(pdata, GLOBAL_CONTROL, 0x000000CD); // GLOBAL_CONTROL
	write_reg(pdata, TIME_BASE_CONTROL, 0x0000004E); // TIME_BASE_CONTROL
	write_reg(pdata, PS_AXI_DMA_PACKETIZER_PACKET_LENGTH, 0x00080000); // AXI_DMA_PACKETIZER/PACKET_LENGTH
	write_reg(pdata, EVT_DATA_FORMATTER_CONTROL, 0x00000000); // EVT_DATA_FORMATTER_CONTROL
	write_reg(pdata, MONITOR_CONTROL_EVT_MERGE_CONTROL, 0x00000003); // CONTROL/EVT_MERGE_CONTROL
	write_reg(pdata, TH_RECOVERY_CONTROL, 0x00000000); // 
	write_reg(pdata, TS_CHECKER_CONTROL, 0x00030D41);
	write_reg(pdata, EVT_MERGE_CONTROL, 0x00000000);
	write_reg(pdata, MIPI_CORE_CONFIG, 0x00000001);
	write_reg(pdata, IF_CONTROL, 0x00400000);
	write_reg(pdata, IF_CONTROL, 0x00400000);
	write_reg(pdata, IF_CONTROL, 0x00400000);
	write_reg(pdata, IF_HUAHINE_CTRL, 0x00000000);
	write_reg(pdata, IF_HUAHINE_CTRL, 0x00000000);
	write_reg(pdata, IF_HUAHINE_CTRL, 0x00000000);
	write_reg(pdata, IF_HUAHINE_CTRL, 0x00000000);
	write_reg(pdata, IF_CONTROL, 0x00400000);
	write_reg(pdata, IF_HUAHINE_CTRL, 0x00000001);
	write_reg(pdata, IF_HUAHINE_CLK_CTRL, 0x01405002); // CHECK: Changes Sensor clock control, sets amount of timei n VCO cycles that clock output remains High and Low
	write_reg(pdata, IF_HUAHINE_CLK_CTRL, 0x01405000);
	/*
		Might need to call mcu to enable CCAM5 
	*/
	// write_reg(pdata, IF_CCAM5_CONTROL, 0x00000000);
	// write_reg(pdata, IF_CCAM5_CONTROL, 0x00000001);
	// msleep_interruptible(500);
	// write_reg(pdata, IF_CCAM5_CONTROL, 0x00000003); // pulls it out of reset
	// msleep_interruptible(500);
	write_reg(pdata, IF_HUAHINE_CTRL, 0x00000001);
	write_reg(pdata, IF_HUAHINE_CTRL, 0x00000001);
	write_reg(pdata, IF_HUAHINE_CLK_CTRL, 0x01405001);
	write_reg(pdata, IF_HUAHINE_CTRL, 0x00000101);
	write_reg(pdata, IF_HUAHINE_CTRL, 0x00000301);

	// Disable MIPI CSI-2
	write_reg(pdata, IMX636_MIPI_CTRL, 0x00000158);
	// Power down CSI-2 and D-PHY
	write_reg(pdata, IMX636_MIPI_STREAM, 0x00000000);
	write_reg(pdata, IMX636_MIPI_ESC_CTRL, 0x0000000A);
	write_reg(pdata, IMX636_MIPI_POWER, 0x00000000);
	write_reg(pdata, IMX636_MIPI_DPHY_POWER, 0x00000000);
	write_reg(pdata, IMX636_MIPI_POWER, 0x00000000);
	write_reg(pdata, IMX636_MIPI_POWER, 0x00000000);

	// Power down PLL
	write_reg(pdata, IMX636_GLOBAL_CTRL, 0x4F006442);
	write_reg(pdata, IMX636_GLOBAL_CTRL, 0x0F006442);
	write_reg(pdata, IMX636_DIVISOR_CTRL, 0x00000400);
	write_reg(pdata, IMX636_DIVISOR_CTRL, 0x00000400);
	write_reg(pdata, IMX636_MIPI_PL_RG_7, 0x00000000);

	// Reconfigure the PLL
	write_reg(pdata, IMX636_MIPI_PL_RG_5, 0x00000002);
	write_reg(pdata, IMX636_MIPI_PL_RG_6, PLL_FB_DIV_D_600);
	write_reg(pdata, IMX636_GLOBAL_CTRL_2, 0x00000108); // These two same as Chris's code
	write_reg(pdata, IMX636_GLOBAL_CTRL_2, 0x00000208); // Seems to like setting one bit at a time

	// Power up PLL
	write_reg(pdata, IMX636_MIPI_BYTECLK_CTRL, 0x00000001);
	write_reg(pdata, IMX636_SLVS_LINKCLK_CTRL, 0x00000000);
	write_reg(pdata, IMX636_MIPI_PL_RG_2, 0x00000004);
	write_reg(pdata, IMX636_MIPI_PL_RG_7, 0x00000001);
	write_reg(pdata, IMX636_MIPI_PL_RG_7, 0x00000003);

	// Some setups
	write_reg(pdata, IMX636_DIVISOR_CTRL, 0x00000401);
	write_reg(pdata, IMX636_DIVISOR_CTRL, 0x00000409);
	write_reg(pdata, IMX636_GLOBAL_CTRL, 0x4F006442);
	write_reg(pdata, IMX636_GLOBAL_CTRL, 0x4F00644A);

	// Update D-PHY timings for new clocking
	write_reg(pdata, IMX636_MIPI_TCLKPOST, TCLKPOST_600);
	write_reg(pdata, IMX636_MIPI_TCLKPRE, TCLKPRE_600);
	write_reg(pdata, IMX636_MIPI_TCLKPREPARE, TCLKPREPARE_600);
	write_reg(pdata, IMX636_MIPI_TCLKTRAIL, TCLKTRAIL_600);
	write_reg(pdata, IMX636_MIPI_TCLKZERO, TCLKZERO_600);
	write_reg(pdata, IMX636_MIPI_THSEXIT, THSEXIT_600);
	write_reg(pdata, IMX636_MIPI_THSPREPARE, THSPREPARE_600);
	write_reg(pdata, IMX636_MIPI_THSZERO, THSZERO_600);
	write_reg(pdata, IMX636_MIPI_THSTRAIL, THSTRAIL_600);
	write_reg(pdata, IMX636_MIPI_TLPX, TLPX_600);
	write_reg(pdata, IMX636_MIPI_TXCLKESC_FREQ, TXCLKESC_FREQ_600);
	write_reg(pdata, IMX636_MIPI_DPHY_PLL_DIV, DPHY_CLK_DIV_600);

	// Think this is analog stuff
	write_reg(pdata, 0x0010A000, 0x000B0501);
	write_reg(pdata, 0x0010A008, 0x00002405);
	write_reg(pdata, 0x0010A004, 0x000B0501);
	write_reg(pdata, 0x0010A020, 0x00000150);

	// Power up D-PHY
	write_reg(pdata, IMX636_MIPI_POWER, 0x00000007);
	write_reg(pdata, IMX636_MIPI_PL_RG_1, 0x00000006);
	write_reg(pdata, IMX636_MIPI_POWER, 0x0000000F);
	write_reg(pdata, IMX636_MIPI_ESC_CTRL, 0x0000008A);
	write_reg(pdata, IMX636_MIPI_DPHY_POWER, 0x00000003);

	// Re-enable stream and control
	write_reg(pdata, IMX636_MIPI_STREAM, 0x00000001);
	write_reg(pdata, IMX636_MIPI_CTRL, 0x00000159);

	// who knows
	write_reg(pdata, 0x00107008, 0x00000001);

	// Apply EVT 3.0 Format
	write_reg(pdata, IMX636_EDF_PIPE_CTRL, 0x00070001);
	write_reg(pdata, IMX636_EOI_PIPE_CTRL, 0x0001E085);

	// Disable timebase
	write_reg(pdata, IMX636_RO_TIME_BASE_CTRL, 0x0000064A);

	// who knows
	write_reg(pdata, 0x00100044, 0xCCFFCCCF);

	// Pixel reset
	write_reg(pdata, IMX636_ROI_CTRL, 0xF0005042);

	// Set Pixel monitor reference current control to Pmos leak
	write_reg(pdata, IMX636_SPARE_CTRL_0, 0x00000200);

	// who knows
	write_reg(pdata, 0x00101014, 0x11A1504D);
	write_reg(pdata, 0x00109004, 0x00000000);

	// Disable RO pattern
	write_reg(pdata, IMX636_RO_CTRL, 0x00000200);

	/*
	 * Try to configure with default parameters. Notice: this is the
	 * very first open, so, we cannot race against other calls,
	 * apart from someone else calling open() simultaneously, but
	 * .host_lock is protecting us against it.
	 */
	ret = psee_s_fmt_vid_cap(file, NULL, &f);
	if (ret < 0)
		goto esfmt;
	return 0;
esfmt:
	return ret;
}

static int psee_video_open(struct file *file)
{
	struct psee_video *pdata = video_drvdata(file);
	int ret;

	mutex_lock(&pdata->lock);

	file->private_data = pdata;

	ret = v4l2_fh_open(file);
	if (ret)
		goto unlock;

	if (!v4l2_fh_is_singular_file(file))
		goto unlock;

	if (psee_video_initialize_device(file)) {
		v4l2_fh_release(file);
		ret = -ENODEV;
	}

unlock:
	mutex_unlock(&pdata->lock);
	return ret;
}

static int psee_video_release(struct file *file)
{
	struct psee_video *pdata = video_drvdata(file);
	bool fh_singular;
	int ret;

	mutex_lock(&pdata->lock);

	/* Save the singular status before we call the clean-up helper */
	fh_singular = v4l2_fh_is_singular_file(file);

	/* the release helper will cleanup any on-going streaming */
	ret = _vb2_fop_release(file, NULL);

	/*
	 * If this was the last open file.
	 * Then de-initialize hw module.
	 */
	if (fh_singular) {
		// Analog DESTROY
		write_reg(pdata, 0x00100070, 0x00400008);
		write_reg(pdata, 0x0010006C, 0x0EE47114);
		write_reg(pdata, 0x0010A00C, 0x00020400);
		write_reg(pdata, 0x0010A010, 0x00008068);
		write_reg(pdata, 0x00101104, 0x00000000);
		write_reg(pdata, 0x0010A020, 0x00000050);
		write_reg(pdata, 0x0010A004, 0x000B0500);
		write_reg(pdata, 0x0010A008, 0x00002404);
		write_reg(pdata, 0x0010A000, 0x000B0500);
		// Digital DESTROY
		write_reg(pdata, IMX636_MIPI_STREAM, 0x00000000);
		write_reg(pdata, IMX636_MIPI_ESC_CTRL, 0x0000000A);
		write_reg(pdata, IMX636_MIPI_POWER, 0x0000000E);
		write_reg(pdata, IMX636_MIPI_DPHY_POWER, 0x00000000);
		write_reg(pdata, IMX636_MIPI_POWER, 0x00000006);
		write_reg(pdata, IMX636_MIPI_POWER, 0x00000004);
		write_reg(pdata, IMX636_GLOBAL_CTRL, 0x4F006442);
		write_reg(pdata, IMX636_GLOBAL_CTRL, 0x0F006442);
		write_reg(pdata, IMX636_DIVISOR_CTRL, 0x00000401);
		write_reg(pdata, IMX636_DIVISOR_CTRL, 0x00000400);
		write_reg(pdata, IMX636_MIPI_PL_RG_7, 0x00000000);
		write_reg(pdata, IF_HUAHINE_CTRL, 0x00000201);
		write_reg(pdata, IF_HUAHINE_CTRL, 0x00000001);
		write_reg(pdata, IF_HUAHINE_CLK_CTRL, 0x01405000);
		write_reg(pdata, IF_CCAM5_CONTROL, 0x00000001);
		write_reg(pdata, IF_CCAM5_CONTROL, 0x00000000);
		write_reg(pdata, CLK_CONTROL, 0x00000777);
		write_reg(pdata, CLK_CONTROL, 0x00010111);
		write_reg(pdata, CLK_CONTROL, 0x00000000);
	}

	mutex_unlock(&pdata->lock);

	return ret;
}

static const struct v4l2_file_operations psee_video_fops = {
	.owner		= THIS_MODULE,
	.open		= psee_video_open,
	.release	= psee_video_release,
	.unlocked_ioctl	= video_ioctl2,
	.poll		= vb2_fop_poll,
	.mmap		= vb2_fop_mmap,
	.read		= vb2_fop_read,
};

static int psee_querycap(struct file *file, void *priv,
			 struct v4l2_capability *cap)
{
	struct psee_video *pdata = video_drvdata(file);

	strscpy(cap->driver, KBUILD_MODNAME, sizeof(cap->driver));
	strscpy(cap->card, pdata->mdev.model, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 pdata->vdev.name);
	return 0;
}

static int psee_enum_input(struct file *file, void *priv,
			   struct v4l2_input *i)
{
	if (i->index != 0)
		return -EINVAL;

	i->type = V4L2_INPUT_TYPE_CAMERA;
	i->capabilities = 0;
	i->std = 0;

	strscpy(i->name, "CCAM5", sizeof(i->name));

	return 0;
}

static int psee_g_input(struct file *file, void *priv, unsigned int *i)
{
	*i = 0;
	return 0;
}

static int psee_s_input(struct file *file, void *priv, unsigned int i)
{
	if (i > 0)
		return -EINVAL;
	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int psee_g_chip_info(struct file *file, void *priv,
			      struct v4l2_dbg_chip_info *chip)
{
	struct psee_video *pdata = video_drvdata(file);

	if ((chip->match.type != V4L2_CHIP_MATCH_BRIDGE) &&
		(chip->match.type != V4L2_CHIP_MATCH_SUBDEV))
		return -EINVAL;

	if (chip->match.addr != 0)
		return -EINVAL;

	strscpy(chip->name, pdata->v4l2_dev.name, sizeof(chip->name));
	return 0;
}

static int psee_g_register(struct file *file, void *fh, struct v4l2_dbg_register *reg)
{
	struct psee_video *pdata = video_drvdata(file);

	if ((reg->match.type != V4L2_CHIP_MATCH_BRIDGE) &&
		(reg->match.type != V4L2_CHIP_MATCH_SUBDEV))
		return -EINVAL;

	if (reg->match.addr != 0)
		return -EINVAL;

	reg->val = read_reg(pdata, reg->reg & 0xfffffc);
	reg->size = 4;
	return 0;
}

static int psee_s_register(struct file *file, void *fh, const struct v4l2_dbg_register *reg)
{
	struct psee_video *pdata = video_drvdata(file);

	if ((reg->match.type != V4L2_CHIP_MATCH_BRIDGE) &&
		(reg->match.type != V4L2_CHIP_MATCH_SUBDEV))
		return -EINVAL;

	if (reg->match.addr != 0)
		return -EINVAL;

	write_reg(pdata, reg->reg & 0xfffffc, reg->val);
	return 0;
}
#endif

static const struct v4l2_ioctl_ops psee_video_ioctl_ops = {
	.vidioc_querycap		= psee_querycap,
	.vidioc_try_fmt_vid_cap		= psee_try_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap		= psee_g_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap		= psee_s_fmt_vid_cap,
	.vidioc_enum_fmt_vid_cap	= psee_enum_fmt_vid_cap,

	.vidioc_enum_input		= psee_enum_input,
	.vidioc_g_input			= psee_g_input,
	.vidioc_s_input			= psee_s_input,

	.vidioc_reqbufs			= vb2_ioctl_reqbufs,
	.vidioc_create_bufs		= vb2_ioctl_create_bufs,
	.vidioc_querybuf		= vb2_ioctl_querybuf,
	.vidioc_qbuf			= vb2_ioctl_qbuf,
	.vidioc_dqbuf			= vb2_ioctl_dqbuf,
	.vidioc_expbuf			= vb2_ioctl_expbuf,
	.vidioc_prepare_buf		= vb2_ioctl_prepare_buf,
	.vidioc_streamon		= vb2_ioctl_streamon,
	.vidioc_streamoff		= vb2_ioctl_streamoff,

#ifdef CONFIG_VIDEO_ADV_DEBUG
	.vidioc_g_chip_info		= psee_g_chip_info,
	.vidioc_g_register		= psee_g_register,
	.vidioc_s_register		= psee_s_register,
#endif
};

/*
 * Setup the constraints of the queue: besides setting the number of planes
 * per buffer and the size and allocation context of each plane, it also
 * checks if sufficient buffers have been allocated. Usually 3 is a good
 * minimum number: many DMA engines need a minimum of 2 buffers in the
 * queue and you need to have another available for userspace processing.
 */
static int queue_setup(struct vb2_queue *vq,
		       unsigned int *nbuffers, unsigned int *nplanes,
		       unsigned int sizes[], struct device *alloc_devs[])
{
	if (*nplanes)
		return sizes[0] < SIZE_IMAGE ? -EINVAL : 0;
	*nplanes = 1;
	sizes[0] = SIZE_IMAGE;
	return 0;
}

static int buf_init(struct vb2_buffer *vb)
{
	struct psee_buffer *buf = to_psee_buffer(vb);

	INIT_LIST_HEAD(&buf->list);

	return 0;
}

/*
 * Prepare the buffer for queueing to the DMA engine: check and set the
 * payload size.
 */
static int buffer_prepare(struct vb2_buffer *vb)
{
	struct psee_video *pdata = vb2_get_drv_priv(vb->vb2_queue);

	if (vb2_plane_size(vb, 0) < SIZE_IMAGE) {
		dev_err(&pdata->vdev.dev, "buffer too small (%lu < %lu)\n",
			 vb2_plane_size(vb, 0), SIZE_IMAGE);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, SIZE_IMAGE);
	return 0;
}

static void dma_callback(void *param)
{
	struct psee_buffer *buf = (struct psee_buffer *)param;
	struct psee_video *pdata = vb2_get_drv_priv(buf->vb.vb2_buf.vb2_queue);
	struct dma_tx_state state;
	enum dma_status status;
	unsigned long flags;

	spin_lock_irqsave(&pdata->qlock, flags);

	/* Check DMA status */
	status = dmaengine_tx_status(pdata->chan[OUT], buf->dma_cookie, &state);

	switch (status) {
	case DMA_IN_PROGRESS:
		dev_dbg(pdata->mdev.dev, "%s: Received DMA_IN_PROGRESS\n", __func__);
		break;
	case DMA_PAUSED:
		dev_err(pdata->mdev.dev, "%s: Received DMA_PAUSED\n", __func__);
		break;
	case DMA_ERROR:
		dev_err(pdata->mdev.dev, "%s: Received DMA_ERROR\n", __func__);
		/* Return buffer to V4L2 in error state */
		fallthrough;
	case DMA_COMPLETE:
		dev_dbg(pdata->mdev.dev, "%s: Received DMA_COMPLETE\n", __func__);

		/* Return buffer to V4L2 */
		list_del_init(&buf->list);
		buf->vb.sequence = pdata->sequence++;
		buf->vb.field = V4L2_FIELD_NONE;
		buf->vb.vb2_buf.timestamp = ktime_get_ns();
		vb2_set_plane_payload(&buf->vb.vb2_buf, 0, SIZE_IMAGE - state.residue);
		vb2_buffer_done(&buf->vb.vb2_buf,
			(status == DMA_COMPLETE) ? VB2_BUF_STATE_DONE : VB2_BUF_STATE_ERROR);
		dev_dbg(pdata->mdev.dev, "buffer[%d] done seq=%d\n",
			buf->vb.vb2_buf.index, buf->vb.sequence);
		break;
	default:
		dev_err(pdata->mdev.dev, "%s: Received unknown status\n", __func__);
		break;
	}

	spin_unlock_irqrestore(&pdata->qlock, flags);
}

/*
 * Queue this buffer to the DMA engine.
 */
static void buffer_queue(struct vb2_buffer *vb)
{
	struct psee_video *pdata = vb2_get_drv_priv(vb->vb2_queue);
	struct psee_buffer *buf = to_psee_buffer(vb);
	unsigned long flags;
	struct dma_async_tx_descriptor *desc = NULL;

	spin_lock_irqsave(&pdata->qlock, flags);
	list_add_tail(&buf->list, &pdata->buffers);

	/* Prepare a DMA transaction */
	desc = dmaengine_prep_slave_single(pdata->chan[OUT],
			vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 0),
			vb2_plane_size(&buf->vb.vb2_buf, 0),
			DMA_DEV_TO_MEM,
			DMA_PREP_INTERRUPT);
	if (!desc) {
		dev_err(pdata->mdev.dev, "%s: DMA prep_single failed: phy=%llu size=%zu\n",
			__func__,
			vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 0),
			vb2_plane_size(&buf->vb.vb2_buf, 0));
		goto release_spinlock;
	}

	/* Set completion callback routine for notification */
	desc->callback = dma_callback;
	desc->callback_param = buf;

	/* Push current DMA transaction in the pending queue */
	buf->dma_cookie = dmaengine_submit(desc);
	if (dma_submit_error(buf->dma_cookie)) {
		dev_err(pdata->mdev.dev, "%s: DMA submission failed\n", __func__);
		goto release_spinlock;
	}

	dma_async_issue_pending(pdata->chan[OUT]);

release_spinlock:
	spin_unlock_irqrestore(&pdata->qlock, flags);
}

static void return_all_buffers(struct psee_video *pdata,
			       enum vb2_buffer_state state)
{
	struct psee_buffer *buf, *node;
	unsigned long flags;

	spin_lock_irqsave(&pdata->qlock, flags);
	list_for_each_entry_safe(buf, node, &pdata->buffers, list) {
		vb2_buffer_done(&buf->vb.vb2_buf, state);
		list_del(&buf->list);
	}
	spin_unlock_irqrestore(&pdata->qlock, flags);
}

/*
 * Start streaming. First check if the minimum number of buffers have been
 * queued. If not, then return -ENOBUFS and the vb2 framework will call
 * this function again the next time a buffer has been queued until enough
 * buffers are available to actually start the DMA engine.
 */
static int start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct psee_video *pdata = vb2_get_drv_priv(vq);
	int ret = 0;
	u32 val;

	pdata->sequence = 0;

	// Enable and bypass TH Recovery block
	write_reg(pdata, TH_RECOVERY_CONTROL, 0x00000003);
	// Enable event data formatter
	write_reg(pdata, EVT_DATA_FORMATTER_CONTROL, 0x00000001);
	// Enable Moorea data handler and sets GEN_LAST
	write_reg(pdata, IF_CONTROL, 0x00400001);
	// Enable CSI on IMX636
	write_reg(pdata, IMX636_MIPI_CTRL, 0x00000159);
	// not sure
	write_reg(pdata, 0x00109028, 0x00000000);
	// Enable RO time base, mode = external, us counter max 100
	write_reg(pdata, IMX636_RO_TIME_BASE_CTRL, 0x0000064B);
	// Analog START
	write_reg(pdata, 0x0010002C, 0x0022C724);
	// Makes sure pixels are active
	val = read_reg(pdata, IMX636_ROI_CTRL);
	val |= 0x400;
	write_reg(pdata, IMX636_ROI_CTRL, val);
	// Enable and bypass event merge
	write_reg(pdata, EVT_MERGE_CONTROL, 0x00000003);
	// Enable time base generation
	val = read_reg(pdata, TIME_BASE_CONTROL);
	val |= 0x1;
	write_reg(pdata, TIME_BASE_CONTROL, val);

	if (ret) {
		/*
		 * In case of an error, return all active buffers to the
		 * QUEUED state
		 */
		return_all_buffers(pdata, VB2_BUF_STATE_QUEUED);
	}
	return ret;
}

/*
 * Stop the DMA engine. Any remaining buffers in the DMA queue are dequeued
 * and passed on to the vb2 framework marked as STATE_ERROR.
 */
static void stop_streaming(struct vb2_queue *vq)
{
	struct psee_video *pdata = vb2_get_drv_priv(vq);
	u32 val;

	write_reg(pdata, EVT_MERGE_CONTROL, 0x00000000);
	write_reg(pdata, TIME_BASE_CONTROL, 0x0000004E);
	val = read_reg(pdata, TIME_BASE_CONTROL);
	val &= ~0x1;
	write_reg(pdata, TIME_BASE_CONTROL, val);

	// Analog STOP
	val = read_reg(pdata, IMX636_ROI_CTRL);
	val &= ~0x400;
	write_reg(pdata, IMX636_ROI_CTRL, val);
	write_reg(pdata, 0x0010002C, 0x0022C324);
	// Digital STOP
	write_reg(pdata, 0x00109028, 0x00000002);
	write_reg(pdata, IMX636_RO_TIME_BASE_CTRL, 0x0000064A);
	write_reg(pdata, IMX636_MIPI_CTRL, 0x00000158);

	dmaengine_terminate_sync(pdata->chan[OUT]);

	/* Release all active buffers */
	return_all_buffers(pdata, VB2_BUF_STATE_ERROR);
}

static const struct vb2_ops psee_qops = {
	.queue_setup		= queue_setup,
	.buf_init		= buf_init,
	.buf_prepare		= buffer_prepare,
	.buf_queue		= buffer_queue,
	.start_streaming	= start_streaming,
	.stop_streaming		= stop_streaming,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
};

static int psee_video_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct psee_video *pdata;
	int rc = 0;
	u32 systemID;

	dev_info(dev, "Probing\n");
	pdata = devm_kzalloc(&pdev->dev,
			sizeof(struct psee_video),
			GFP_KERNEL);
	if (!pdata) {
		dev_err(&pdev->dev, "Could not allocate private data\n");
		return -ENOMEM;
	}

	pdata->reg_resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!pdata->reg_resource) {
		dev_err(&pdev->dev, "Could not get reg resource\n");
		return -ENXIO;
	}

	pdata->regmap = devm_ioremap_resource(&pdev->dev,
					pdata->reg_resource);
	if (!pdata->regmap) {
		dev_err(&pdev->dev, "Could not map regbank\n");
		return -ENXIO;
	}

	systemID = read_reg(pdata, SYSTEM_ID);
	if ((systemID != 0x32) && (systemID != 0x3A)) {
		dev_err(dev, "FPGA reported unknown ID: 0x%x\n", systemID);
		return -ENODEV;
	}

	mutex_init(&pdata->lock);

	media_device_init(&pdata->mdev);
	pdata->mdev.dev = dev;
	strncpy(pdata->mdev.model, "Prophesee Event-Based Video IP",
		sizeof(pdata->mdev.model));

	pdata->v4l2_dev.mdev = &pdata->mdev;
	rc = v4l2_device_register(dev, &pdata->v4l2_dev);
	if (rc < 0) {
		dev_err(dev, "V4L2 device registration failed (%d)\n", rc);
		goto cleanup_media;
	}

	pdata->chan[OUT] = dma_request_chan(dev, "output");
	if (IS_ERR(pdata->chan[OUT])) {
		rc = PTR_ERR(pdata->chan[OUT]);
		dev_err(dev, "DMA chan \"output\" request failed (%d)\n", rc);
		goto unregister_v4l2;
	}

	pdata->chan[IN] = dma_request_chan(dev, "input");
	if (IS_ERR(pdata->chan[IN])) {
		rc = PTR_ERR(pdata->chan[IN]);
		dev_err(dev, "DMA chan \"input\" request failed (%d)\n", rc);
		goto release_output;
	}

	/* for the DMA engine */
	INIT_LIST_HEAD(&pdata->buffers);
	spin_lock_init(&pdata->qlock);

	/* buffer queue */
	pdata->queue.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	pdata->queue.io_modes = VB2_MMAP | VB2_READ | VB2_DMABUF;
	pdata->queue.lock = &pdata->lock;
	pdata->queue.drv_priv = pdata;
	pdata->queue.buf_struct_size = sizeof(struct psee_buffer);
	pdata->queue.ops = &psee_qops;
	pdata->queue.mem_ops = &vb2_dma_contig_memops;
	pdata->queue.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	/* issues were seen below 4 buffers, to be investigated */
	pdata->queue.min_buffers_needed = 4;
	pdata->queue.dev = dev;

	rc = vb2_queue_init(&pdata->queue);
	if (rc < 0) {
		dev_err(dev, "failed to initialize VB2 queue\n");
		goto release_input;
	}

	strscpy(pdata->vdev.name, "psee-video", sizeof(pdata->vdev.name));
	pdata->vdev.fops = &psee_video_fops;
	pdata->vdev.ioctl_ops = &psee_video_ioctl_ops;
	pdata->vdev.minor = -1;
	pdata->vdev.release = video_device_release_empty;
	pdata->vdev.lock = &pdata->lock;
	pdata->vdev.v4l2_dev = &pdata->v4l2_dev;
	pdata->vdev.queue = &pdata->queue;
	pdata->vdev.vfl_dir = VFL_DIR_RX;
	pdata->vdev.device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING | V4L2_CAP_READWRITE;
	video_set_drvdata(&pdata->vdev, pdata);

	rc = video_register_device(&pdata->vdev, VFL_TYPE_VIDEO, -1);
	if (rc) {
		dev_err(dev, "Failed to register video device\n");
		goto release_queue;
	}


	dev_set_drvdata(dev, pdata);

	rc = media_device_register(&pdata->mdev);
	if (rc < 0)
		goto release_video;

	dev_info(dev, "Device probed\n");
	return rc;

release_video:
	video_unregister_device(&pdata->vdev);
release_queue:
	vb2_queue_release(&pdata->queue);
release_input:
	dma_release_channel(pdata->chan[IN]);
release_output:
	dma_release_channel(pdata->chan[OUT]);
unregister_v4l2:
	v4l2_device_unregister(&pdata->v4l2_dev);
cleanup_media:
	media_device_cleanup(&pdata->mdev);
	mutex_destroy(&pdata->lock);
	return rc;
}

static int psee_video_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct psee_video *pdata = dev_get_drvdata(dev);

	dev_info(dev, "Removing driver\n");
	media_device_unregister(&pdata->mdev);
	dev_set_drvdata(dev, NULL);
	dma_release_channel(pdata->chan[IN]);
	dma_release_channel(pdata->chan[OUT]);
	v4l2_device_unregister(&pdata->v4l2_dev);
	media_device_cleanup(&pdata->mdev);
	mutex_destroy(&pdata->lock);
	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id psee_video_of_match[] = {
	{ .compatible = "psee,video", },
	{},
};
MODULE_DEVICE_TABLE(of, psee_video_of_match);
#else
# define psee_video_of_match
#endif


static struct platform_driver psee_video_driver = {
	.driver = {
		.name = "psee-video",
		.owner = THIS_MODULE,
		.of_match_table	= psee_video_of_match,
	},
	.probe		= psee_video_probe,
	.remove		= psee_video_remove,
};

static int __init psee_video_init(void)
{
	return platform_driver_register(&psee_video_driver);
}


static void __exit psee_video_exit(void)
{
	platform_driver_unregister(&psee_video_driver);
}

module_init(psee_video_init);
module_exit(psee_video_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Prophesee");
MODULE_DESCRIPTION("psee-video - media/v4l2 driver for Prophesee video IP");
