#include "pinctrl-common.h"

/* pin_number  ctrl_pin bit_offset bit_width */

/* pin_ctrl_reg0 */
#define SC9861_ORP_URXD_PIN_IN_SEL	SPRD_PIN_INFO(0, 1, 27, 1, 0)
#define SC9861_WPD_IO_1_4PD		SPRD_PIN_INFO(1, 1, 20, 1, 0)
#define SC9861_WPD_IO_1_3PD		SPRD_PIN_INFO(2, 1, 19, 1, 0)
#define SC9861_WPD_IO_1_2PD		SPRD_PIN_INFO(3, 1, 18, 1, 0)
#define SC9861_WPD_IO_1_1PD		SPRD_PIN_INFO(4, 1, 17, 1, 0)
#define SC9861_WPD_IO_1_0PD		SPRD_PIN_INFO(5, 1, 16, 1, 0)
#define SC9861_WPD_NF1PD		SPRD_PIN_INFO(6, 1, 15, 1, 0)
#define SC9861_WPD_NF0PD		SPRD_PIN_INFO(7, 1, 14, 1, 0)
#define SC9861_WPD_ADPD		SPRD_PIN_INFO(8, 1, 13, 1, 0)
#define SC9861_WPD_IO_2_1PD		SPRD_PIN_INFO(9, 1, 12, 1, 0)
#define SC9861_WPD_SIM2PD		SPRD_PIN_INFO(10, 1, 10, 1, 0)
#define SC9861_WPD_SIM1PD		SPRD_PIN_INFO(11, 1, 9, 1, 0)
#define SC9861_WPD_SIM0PD		SPRD_PIN_INFO(12, 1, 8, 1, 0)
#define SC9861_WPD_SDPD		SPRD_PIN_INFO(13, 1, 7, 1, 0)
#define SC9861_WPD_CAMPD		SPRD_PIN_INFO(14, 1, 6, 1, 0)



/* pin_ctrl_reg1 */
#define SC9861_SD1_D1_CTRL		SPRD_PIN_INFO(15, 1, 31, 1, 1)
#define SC9861_SIMDA2_CTRL		SPRD_PIN_INFO(16, 1, 30, 1, 1)
#define SC9861_SDA1_CTRL		SPRD_PIN_INFO(17, 1, 29, 1, 1)
#define SC9861_U1TXD_CTRL	SPRD_PIN_INFO(18, 1, 28, 1, 1)
#define SC9861_SL_SIM2		SPRD_PIN_INFO(19, 1, 15, 1, 1)
#define SC9861_SL_SIM1		SPRD_PIN_INFO(20, 1, 14, 1, 1)
#define SC9861_SL_SIM0		SPRD_PIN_INFO(21, 1, 13, 1, 1)
#define SC9861_SL_SD		SPRD_PIN_INFO(22, 1, 12, 1, 1)
#define SC9861_MS_SD		SPRD_PIN_INFO(23, 1, 11, 1, 1)
#define SC9861_SPSPI_CLK_PIN_IN_SEL		SPRD_PIN_INFO(24, 1, 10, 1, 1)
#define SC9861_SPSPI_CSN_PIN_IN_SEL		SPRD_PIN_INFO(25, 1, 9, 1, 1)
#define SC9861_SPSPI_DO_PIN_IN_SEL		SPRD_PIN_INFO(26, 1, 8, 1, 1)
#define SC9861_SPSPI_DI_PIN_IN_SEL		SPRD_PIN_INFO(27, 1, 7, 1, 1)
#define SC9861_ADI_SYNC_PIN_OUT_SEL		SPRD_PIN_INFO(28, 1, 6, 1, 1)
#define SC9861_CMRST_SEL		SPRD_PIN_INFO(29, 1, 5, 1, 1)
#define SC9861_CMPD_SEL		SPRD_PIN_INFO(30, 1, 4, 1, 1)

/* pin_ctrl_reg2 */
#define SC9861_UART5_SYS_SEL		SPRD_PIN_INFO(31, 1, 24, 3, 2)
#define SC9861_SIM2_SYS_SEL		SPRD_PIN_INFO(32, 1, 23, 1, 2)
#define SC9861_SIM1_SYS_SEL		SPRD_PIN_INFO(33, 1, 22, 1, 2)
#define SC9861_SIM0_SYS_SEL		SPRD_PIN_INFO(34, 1, 21, 1, 2)
#define SC9861_UART1_SYS_SEL		SPRD_PIN_INFO(35, 1, 18, 3, 2)
#define SC9861_UART4_SYS_SEL		SPRD_PIN_INFO(36, 1, 15, 3, 2)
#define SC9861_UART3_SYS_SEL		SPRD_PIN_INFO(37, 1, 12, 3, 2)
#define SC9861_UART2_SYS_SEL		SPRD_PIN_INFO(38, 1, 9, 3, 2)
#define SC9861_UART0_SYS_SEL		SPRD_PIN_INFO(39, 1, 4, 3, 2)
#define SC9861_UART24_LOOP_SEL		SPRD_PIN_INFO(40, 1, 3, 1, 2)
#define SC9861_UART23_LOOP_SEL		SPRD_PIN_INFO(41, 1, 2, 1, 2)
#define SC9861_UART14_LOOP_SEL		SPRD_PIN_INFO(42, 1, 1, 1, 2)
#define SC9861_UART13_LOOP_SEL		SPRD_PIN_INFO(43, 1, 0, 1, 2)

/* pin_ctrl_reg3 */
#define SC9861_WDRST_OUT_SEL		SPRD_PIN_INFO(44, 1, 21, 3, 3)
#define SC9861_IIS2_SYS_SEL		SPRD_PIN_INFO(45, 1, 14, 4, 3)
#define SC9861_IIS1_SYS_SEL		SPRD_PIN_INFO(46, 1, 10, 4, 3)
#define SC9861_IIS0_SYS_SEL		SPRD_PIN_INFO(47, 1, 6, 4, 3)
#define SC9861_IIS23_LOOP_SEL		SPRD_PIN_INFO(48, 1, 5, 1, 3)
#define SC9861_IIS13_LOOP_SEL		SPRD_PIN_INFO(49, 1, 4, 1, 3)
#define SC9861_IIS12_LOOP_SEL		SPRD_PIN_INFO(50, 1, 3, 1, 3)
#define SC9861_IIS03_LOOP_SEL		SPRD_PIN_INFO(51, 1, 2, 1, 3)
#define SC9861_IIS02_LOOP_SEL		SPRD_PIN_INFO(52, 1, 1, 1, 3)
#define SC9861_IIS01_LOOP_SEL		SPRD_PIN_INFO(53, 1, 0, 1, 3)

/* pin_ctrl_reg4 */
#define SC9861_IIS6_SYS_SEL		SPRD_PIN_INFO(54, 1, 27, 4, 4)
#define SC9861_IIS5_SYS_SEL		SPRD_PIN_INFO(55, 1, 23, 4, 4)
#define SC9861_IIS4_SYS_SEL		SPRD_PIN_INFO(56, 1, 19, 4, 4)
#define SC9861_IIS3_SYS_SEL		SPRD_PIN_INFO(57, 1, 15, 4, 4)
#define SC9861_I2C_INF4_SYS_SEL		SPRD_PIN_INFO(58, 1, 13, 2, 4)
#define SC9861_I2C_INF3_SYS_SEL		SPRD_PIN_INFO(59, 1, 11, 2, 4)
#define SC9861_I2C_INF2_SYS_SEL		SPRD_PIN_INFO(60, 1, 9, 2, 4)
#define SC9861_I2C_INF1_SYS_SEL		SPRD_PIN_INFO(61, 1, 7, 2, 4)
#define SC9861_VSD1_MS		SPRD_PIN_INFO(62, 1, 6, 1, 4)
#define SC9861_VSD_MS		SPRD_PIN_INFO(63, 1, 5, 1, 4)
#define SC9861_VSIM2_MS		SPRD_PIN_INFO(64, 1, 4, 1, 4)
#define SC9861_VSIM1_MS		SPRD_PIN_INFO(65, 1, 3, 1, 4)
#define SC9861_VSIM0_MS		SPRD_PIN_INFO(66, 1, 2, 1, 4)
#define SC9861_I2C_INF0_SYS_SEL		SPRD_PIN_INFO(67, 1, 0, 2, 4)

/* pin_ctrl_reg5 */
#define SC9861_GPIO_INF7_SYS_SEL	SPRD_PIN_INFO(68, 1, 27, 1, 5)
#define SC9861_GPIO_INF6_SYS_SEL	SPRD_PIN_INFO(69, 1, 26, 1, 5)
#define SC9861_GPIO_INF5_SYS_SEL	SPRD_PIN_INFO(70, 1, 25, 1, 5)
#define SC9861_GPIO_INF4_SYS_SEL	SPRD_PIN_INFO(71, 1, 24, 1, 5)
#define SC9861_GPIO_INF3_SYS_SEL	SPRD_PIN_INFO(72, 1, 23, 1, 5)
#define SC9861_GPIO_INF2_SYS_SEL	SPRD_PIN_INFO(73, 1, 22, 1, 5)
#define SC9861_GPIO_INF1_SYS_SEL	SPRD_PIN_INFO(74, 1, 21, 1, 5)
#define SC9861_GPIO_INF0_SYS_SEL	SPRD_PIN_INFO(75, 1, 20, 1, 5)

#define SC9861_CARD_DET_SEL		SPRD_PIN_INFO(76, 1, 17, 3, 5)
#define SC9861_SIM0_DET_SEL		SPRD_PIN_INFO(77, 1, 16, 1, 5)
#define SC9861_AP_SIM0_BD_EB		SPRD_PIN_INFO(78, 1, 15, 1, 5)
#define SC9861_AP_EMMC_BD_EB			SPRD_PIN_INFO(79, 1, 14, 1, 5)
#define SC9861_AP_SDIO2_BD_EB		SPRD_PIN_INFO(80, 1, 13, 1, 5)
#define SC9861_AP_SDIO1_BD_EB		SPRD_PIN_INFO(81, 1, 12, 1, 5)
#define SC9861_AP_SDIO0_BD_EB		SPRD_PIN_INFO(82, 1, 11, 1, 5)
#define SC9861_PUBCP_SDIO0_BD_EB		SPRD_PIN_INFO(83, 1, 10, 1, 5)
#define SC9861_PUBCP_SIM1_BD_EB		SPRD_PIN_INFO(84, 1, 9, 1, 5)
#define SC9861_PUBCP_SIM0_BD_EB		SPRD_PIN_INFO(85, 1, 8, 1, 5)
#define SC9861_TEST_DBG_MODE4		SPRD_PIN_INFO(86, 1, 4, 1, 5)
#define SC9861_TEST_DBG_MODE3		SPRD_PIN_INFO(87, 1, 3, 1, 5)
#define SC9861_TEST_DBG_MODE2		SPRD_PIN_INFO(88, 1, 2, 1, 5)
#define SC9861_TEST_DBG_MODE1		SPRD_PIN_INFO(89, 1, 1, 1, 5)
#define SC9861_TEST_DBG_MODE0		SPRD_PIN_INFO(90, 1, 0, 1, 5)

/* pin_ctrl_reg6 */
#define SC9861_SP_EIC_DPAD3_SEL		SPRD_PIN_INFO(91, 1, 24, 8, 6)
#define SC9861_SP_EIC_DPAD2_SEL		SPRD_PIN_INFO(92, 1, 16, 8, 6)
#define SC9861_SP_EIC_DPAD1_SEL		SPRD_PIN_INFO(93, 1, 8, 8, 6)
#define SC9861_SP_EIC_DPAD0_SEL		SPRD_PIN_INFO(94, 1, 0, 8, 6)

/* pin_ctrl_reg7 */
#define SC9861_SP_EIC_DPAD7_SEL		SPRD_PIN_INFO(95, 1, 24, 8, 7)
#define SC9861_SP_EIC_DPAD6_SEL		SPRD_PIN_INFO(96, 1, 16, 8, 7)
#define SC9861_SP_EIC_DPAD5_SEL		SPRD_PIN_INFO(97, 1, 8, 8, 7)
#define SC9861_SP_EIC_DPAD4_SEL		SPRD_PIN_INFO(98, 1, 0, 8, 7)

/* registers definitions for controller CTL_PIN */
#define SC9861_RFCTL32            SPRD_PIN_INFO(99, 0, 0, 0, 0)
#define SC9861_RFCTL33            SPRD_PIN_INFO(101, 0, 0, 0, 0)
#define SC9861_RFCTL34            SPRD_PIN_INFO(103, 0, 0, 0, 0)
#define SC9861_RFCTL35            SPRD_PIN_INFO(105, 0, 0, 0, 0)
#define SC9861_RFCTL36            SPRD_PIN_INFO(107, 0, 0, 0, 0)
#define SC9861_RFCTL37            SPRD_PIN_INFO(109, 0, 0, 0, 0)
#define SC9861_SPI0_CSN           SPRD_PIN_INFO(111, 0, 0, 0, 0)
#define SC9861_SPI0_DO            SPRD_PIN_INFO(113, 0, 0, 0, 0)
#define SC9861_SPI0_DI            SPRD_PIN_INFO(115, 0, 0, 0, 0)
#define SC9861_SPI0_CLK           SPRD_PIN_INFO(117, 0, 0, 0, 0)
#define SC9861_USB30_CC_SWITCH    SPRD_PIN_INFO(119, 0, 0, 0, 0)
#define SC9861_U1TXD              SPRD_PIN_INFO(121, 0, 0, 0, 0)
#define SC9861_U1RXD              SPRD_PIN_INFO(123, 0, 0, 0, 0)
#define SC9861_IIS1DI             SPRD_PIN_INFO(125, 0, 0, 0, 0)
#define SC9861_IIS1DO             SPRD_PIN_INFO(127, 0, 0, 0, 0)
#define SC9861_IIS1CLK            SPRD_PIN_INFO(129, 0, 0, 0, 0)
#define SC9861_IIS1LRCK           SPRD_PIN_INFO(131, 0, 0, 0, 0)
#define SC9861_U2TXD              SPRD_PIN_INFO(133, 0, 0, 0, 0)
#define SC9861_U2RXD              SPRD_PIN_INFO(135, 0, 0, 0, 0)
#define SC9861_IIS3CLK            SPRD_PIN_INFO(137, 0, 0, 0, 0)
#define SC9861_IIS3LRCK           SPRD_PIN_INFO(139, 0, 0, 0, 0)
#define SC9861_IIS3DI             SPRD_PIN_INFO(141, 0, 0, 0, 0)
#define SC9861_IIS3DO             SPRD_PIN_INFO(143, 0, 0, 0, 0)
#define SC9861_SD2_CMD            SPRD_PIN_INFO(145, 0, 0, 0, 0)
#define SC9861_SD2_D0             SPRD_PIN_INFO(147, 0, 0, 0, 0)
#define SC9861_SD2_D1             SPRD_PIN_INFO(149, 0, 0, 0, 0)
#define SC9861_SD2_CLK            SPRD_PIN_INFO(151, 0, 0, 0, 0)
#define SC9861_SD2_D2             SPRD_PIN_INFO(153, 0, 0, 0, 0)
#define SC9861_SD2_D3             SPRD_PIN_INFO(155, 0, 0, 0, 0)
#define SC9861_SD2_DUMY           SPRD_PIN_INFO(157, 0, 0, 0, 0)
#define SC9861_U4TXD              SPRD_PIN_INFO(159, 0, 0, 0, 0)
#define SC9861_U4RXD              SPRD_PIN_INFO(161, 0, 0, 0, 0)
#define SC9861_DCDC_ARM_EN1       SPRD_PIN_INFO(163, 0, 0, 0, 0)
#define SC9861_SENSOR_HUB_ACTION  SPRD_PIN_INFO(165, 0, 0, 0, 0)
#define SC9861_PTEST              SPRD_PIN_INFO(167, 0, 0, 0, 0)
#define SC9861_ANA_INT            SPRD_PIN_INFO(169, 0, 0, 0, 0)
#define SC9861_EXT_RST_B          SPRD_PIN_INFO(171, 0, 0, 0, 0)
#define SC9861_AUD_SCLK           SPRD_PIN_INFO(173, 0, 0, 0, 0)
#define SC9861_DCDC_ARM_EN0       SPRD_PIN_INFO(175, 0, 0, 0, 0)
#define SC9861_CLK_32K            SPRD_PIN_INFO(177, 0, 0, 0, 0)
#define SC9861_CHIP_SLEEP         SPRD_PIN_INFO(179, 0, 0, 0, 0)
#define SC9861_AUD_ADD0           SPRD_PIN_INFO(181, 0, 0, 0, 0)
#define SC9861_AUD_DAD0           SPRD_PIN_INFO(183, 0, 0, 0, 0)
#define SC9861_AUD_ADD1           SPRD_PIN_INFO(185, 0, 0, 0, 0)
#define SC9861_AUD_DAD1           SPRD_PIN_INFO(187, 0, 0, 0, 0)
#define SC9861_AUD_SYNC           SPRD_PIN_INFO(189, 0, 0, 0, 0)
#define SC9861_ADI_SCLK           SPRD_PIN_INFO(191, 0, 0, 0, 0)
#define SC9861_ADI_D              SPRD_PIN_INFO(193, 0, 0, 0, 0)
#define SC9861_MTCK_ARM           SPRD_PIN_INFO(195, 0, 0, 0, 0)
#define SC9861_MTMS_ARM           SPRD_PIN_INFO(197, 0, 0, 0, 0)
#define SC9861_MTRST_N_ARM        SPRD_PIN_INFO(199, 0, 0, 0, 0)
#define SC9861_MTDO_ARM           SPRD_PIN_INFO(201, 0, 0, 0, 0)
#define SC9861_MTDI_ARM           SPRD_PIN_INFO(203, 0, 0, 0, 0)
#define SC9861_EXTINT16           SPRD_PIN_INFO(205, 0, 0, 0, 0)
#define SC9861_KEYOUT0            SPRD_PIN_INFO(207, 0, 0, 0, 0)
#define SC9861_KEYOUT1            SPRD_PIN_INFO(209, 0, 0, 0, 0)
#define SC9861_KEYOUT2            SPRD_PIN_INFO(211, 0, 0, 0, 0)
#define SC9861_KEYIN0             SPRD_PIN_INFO(213, 0, 0, 0, 0)
#define SC9861_KEYIN1             SPRD_PIN_INFO(215, 0, 0, 0, 0)
#define SC9861_KEYIN2             SPRD_PIN_INFO(217, 0, 0, 0, 0)
#define SC9861_SD0_D0             SPRD_PIN_INFO(219, 0, 0, 0, 0)
#define SC9861_SD0_D1             SPRD_PIN_INFO(221, 0, 0, 0, 0)
#define SC9861_SD0_CLK0           SPRD_PIN_INFO(223, 0, 0, 0, 0)
#define SC9861_SD0_CMD            SPRD_PIN_INFO(225, 0, 0, 0, 0)
#define SC9861_SD0_D2             SPRD_PIN_INFO(227, 0, 0, 0, 0)
#define SC9861_SD0_D3             SPRD_PIN_INFO(229, 0, 0, 0, 0)
#define SC9861_SD0_DUMY           SPRD_PIN_INFO(231, 0, 0, 0, 0)
#define SC9861_SIMCLK2            SPRD_PIN_INFO(233, 0, 0, 0, 0)
#define SC9861_SIMDA2             SPRD_PIN_INFO(235, 0, 0, 0, 0)
#define SC9861_SIMRST2            SPRD_PIN_INFO(237, 0, 0, 0, 0)
#define SC9861_SIMCLK1            SPRD_PIN_INFO(239, 0, 0, 0, 0)
#define SC9861_SIMDA1             SPRD_PIN_INFO(241, 0, 0, 0, 0)
#define SC9861_SIMRST1            SPRD_PIN_INFO(243, 0, 0, 0, 0)
#define SC9861_SIMCLK0            SPRD_PIN_INFO(245, 0, 0, 0, 0)
#define SC9861_SIMDA0             SPRD_PIN_INFO(247, 0, 0, 0, 0)
#define SC9861_SIMRST0            SPRD_PIN_INFO(249, 0, 0, 0, 0)
#define SC9861_EMMC_CLK           SPRD_PIN_INFO(251, 0, 0, 0, 0)
#define SC9861_EMMC_RSTB          SPRD_PIN_INFO(253, 0, 0, 0, 0)
#define SC9861_EMMC_CMD           SPRD_PIN_INFO(255, 0, 0, 0, 0)
#define SC9861_EMMC_D0            SPRD_PIN_INFO(257, 0, 0, 0, 0)
#define SC9861_EMMC_D1            SPRD_PIN_INFO(259, 0, 0, 0, 0)
#define SC9861_EMMC_D2            SPRD_PIN_INFO(261, 0, 0, 0, 0)
#define SC9861_EMMC_D3            SPRD_PIN_INFO(263, 0, 0, 0, 0)
#define SC9861_EMMC_D4            SPRD_PIN_INFO(265, 0, 0, 0, 0)
#define SC9861_EMMC_D5            SPRD_PIN_INFO(267, 0, 0, 0, 0)
#define SC9861_EMMC_D6            SPRD_PIN_INFO(269, 0, 0, 0, 0)
#define SC9861_EMMC_D7            SPRD_PIN_INFO(271, 0, 0, 0, 0)
#define SC9861_EMMC_STROBE        SPRD_PIN_INFO(273, 0, 0, 0, 0)
#define SC9861_EMMC_DUMY          SPRD_PIN_INFO(275, 0, 0, 0, 0)
#define SC9861_SD1_CMD            SPRD_PIN_INFO(277, 0, 0, 0, 0)
#define SC9861_SD1_D0             SPRD_PIN_INFO(279, 0, 0, 0, 0)
#define SC9861_SD1_D1             SPRD_PIN_INFO(281, 0, 0, 0, 0)
#define SC9861_SD1_CLK            SPRD_PIN_INFO(283, 0, 0, 0, 0)
#define SC9861_SD1_D2             SPRD_PIN_INFO(285, 0, 0, 0, 0)
#define SC9861_SD1_D3             SPRD_PIN_INFO(287, 0, 0, 0, 0)
#define SC9861_SD1_DUMY           SPRD_PIN_INFO(289, 0, 0, 0, 0)
#define SC9861_IIS0DI             SPRD_PIN_INFO(291, 0, 0, 0, 0)
#define SC9861_IIS0DO             SPRD_PIN_INFO(293, 0, 0, 0, 0)
#define SC9861_IIS0CLK            SPRD_PIN_INFO(295, 0, 0, 0, 0)
#define SC9861_IIS0LRCK           SPRD_PIN_INFO(297, 0, 0, 0, 0)
#define SC9861_U3TXD              SPRD_PIN_INFO(299, 0, 0, 0, 0)
#define SC9861_U3RXD              SPRD_PIN_INFO(301, 0, 0, 0, 0)
#define SC9861_U3CTS              SPRD_PIN_INFO(303, 0, 0, 0, 0)
#define SC9861_U3RTS              SPRD_PIN_INFO(305, 0, 0, 0, 0)
#define SC9861_U0TXD              SPRD_PIN_INFO(307, 0, 0, 0, 0)
#define SC9861_U0RXD              SPRD_PIN_INFO(309, 0, 0, 0, 0)
#define SC9861_U0CTS              SPRD_PIN_INFO(311, 0, 0, 0, 0)
#define SC9861_U0RTS              SPRD_PIN_INFO(313, 0, 0, 0, 0)
#define SC9861_CLK_AUX0           SPRD_PIN_INFO(315, 0, 0, 0, 0)
#define SC9861_RFCTL39            SPRD_PIN_INFO(317, 0, 0, 0, 0)
#define SC9861_RFCTL38            SPRD_PIN_INFO(319, 0, 0, 0, 0)
#define SC9861_WIFI_COEXIST       SPRD_PIN_INFO(321, 0, 0, 0, 0)
#define SC9861_BEIDOU_COEXIST     SPRD_PIN_INFO(323, 0, 0, 0, 0)
#define SC9861_EXTINT12           SPRD_PIN_INFO(325, 0, 0, 0, 0)
#define SC9861_EXTINT11           SPRD_PIN_INFO(327, 0, 0, 0, 0)
#define SC9861_EXTINT10           SPRD_PIN_INFO(329, 0, 0, 0, 0)
#define SC9861_EXTINT9            SPRD_PIN_INFO(331, 0, 0, 0, 0)
#define SC9861_EXTINT8            SPRD_PIN_INFO(333, 0, 0, 0, 0)
#define SC9861_EXTINT7            SPRD_PIN_INFO(335, 0, 0, 0, 0)
#define SC9861_EXTINT6            SPRD_PIN_INFO(337, 0, 0, 0, 0)
#define SC9861_SDA1               SPRD_PIN_INFO(339, 0, 0, 0, 0)
#define SC9861_SCL1               SPRD_PIN_INFO(341, 0, 0, 0, 0)
#define SC9861_EXTINT1            SPRD_PIN_INFO(343, 0, 0, 0, 0)
#define SC9861_EXTINT0            SPRD_PIN_INFO(345, 0, 0, 0, 0)
#define SC9861_EXTINT5            SPRD_PIN_INFO(347, 0, 0, 0, 0)
#define SC9861_DSI_TE             SPRD_PIN_INFO(349, 0, 0, 0, 0)
#define SC9861_LCM_RSTN           SPRD_PIN_INFO(351, 0, 0, 0, 0)
#define SC9861_PWMA               SPRD_PIN_INFO(353, 0, 0, 0, 0)
#define SC9861_SCL2               SPRD_PIN_INFO(355, 0, 0, 0, 0)
#define SC9861_SDA2               SPRD_PIN_INFO(357, 0, 0, 0, 0)
#define SC9861_CMPD1              SPRD_PIN_INFO(359, 0, 0, 0, 0)
#define SC9861_CMPD0              SPRD_PIN_INFO(361, 0, 0, 0, 0)
#define SC9861_CMRST1             SPRD_PIN_INFO(363, 0, 0, 0, 0)
#define SC9861_CMRST0             SPRD_PIN_INFO(365, 0, 0, 0, 0)
#define SC9861_CMMCLK1            SPRD_PIN_INFO(367, 0, 0, 0, 0)
#define SC9861_CMMCLK             SPRD_PIN_INFO(369, 0, 0, 0, 0)
#define SC9861_RFSEN0             SPRD_PIN_INFO(371, 0, 0, 0, 0)
#define SC9861_RFSCK0             SPRD_PIN_INFO(373, 0, 0, 0, 0)
#define SC9861_RFSDA0             SPRD_PIN_INFO(375, 0, 0, 0, 0)
#define SC9861_RFSEN1             SPRD_PIN_INFO(377, 0, 0, 0, 0)
#define SC9861_RFSCK1             SPRD_PIN_INFO(379, 0, 0, 0, 0)
#define SC9861_RFSDA1             SPRD_PIN_INFO(381, 0, 0, 0, 0)
#define SC9861_RFCTL27            SPRD_PIN_INFO(383, 0, 0, 0, 0)
#define SC9861_RFCTL26            SPRD_PIN_INFO(385, 0, 0, 0, 0)
#define SC9861_RFCTL31            SPRD_PIN_INFO(387, 0, 0, 0, 0)
#define SC9861_RFCTL30            SPRD_PIN_INFO(389, 0, 0, 0, 0)
#define SC9861_RF_LVDS0_ADC_ON    SPRD_PIN_INFO(391, 0, 0, 0, 0)
#define SC9861_RF_LVDS0_DAC_ON    SPRD_PIN_INFO(393, 0, 0, 0, 0)
#define SC9861_RF_LVDS1_ADC_ON    SPRD_PIN_INFO(395, 0, 0, 0, 0)
#define SC9861_RF_LVDS1_DAC_ON    SPRD_PIN_INFO(397, 0, 0, 0, 0)
#define SC9861_SDA4               SPRD_PIN_INFO(399, 0, 0, 0, 0)
#define SC9861_SCL4               SPRD_PIN_INFO(401, 0, 0, 0, 0)
#define SC9861_SDA0               SPRD_PIN_INFO(403, 0, 0, 0, 0)
#define SC9861_SCL0               SPRD_PIN_INFO(405, 0, 0, 0, 0)
#define SC9861_RFCTL29            SPRD_PIN_INFO(407, 0, 0, 0, 0)
#define SC9861_RFCTL28            SPRD_PIN_INFO(409, 0, 0, 0, 0)
#define SC9861_RFFE0_SDA0         SPRD_PIN_INFO(411, 0, 0, 0, 0)
#define SC9861_RFFE0_SCK0         SPRD_PIN_INFO(413, 0, 0, 0, 0)
#define SC9861_RFFE1_SDA0         SPRD_PIN_INFO(415, 0, 0, 0, 0)
#define SC9861_RFFE1_SCK0         SPRD_PIN_INFO(417, 0, 0, 0, 0)
#define SC9861_RFCTL0             SPRD_PIN_INFO(419, 0, 0, 0, 0)
#define SC9861_RFCTL1             SPRD_PIN_INFO(421, 0, 0, 0, 0)
#define SC9861_RFCTL2             SPRD_PIN_INFO(423, 0, 0, 0, 0)
#define SC9861_RFCTL3             SPRD_PIN_INFO(425, 0, 0, 0, 0)
#define SC9861_RFCTL4             SPRD_PIN_INFO(427, 0, 0, 0, 0)
#define SC9861_RFCTL5             SPRD_PIN_INFO(429, 0, 0, 0, 0)
#define SC9861_RFCTL6             SPRD_PIN_INFO(431, 0, 0, 0, 0)
#define SC9861_RFCTL7             SPRD_PIN_INFO(433, 0, 0, 0, 0)
#define SC9861_RFCTL8             SPRD_PIN_INFO(435, 0, 0, 0, 0)
#define SC9861_RFCTL9             SPRD_PIN_INFO(437, 0, 0, 0, 0)
#define SC9861_RFCTL10            SPRD_PIN_INFO(439, 0, 0, 0, 0)
#define SC9861_RFCTL11            SPRD_PIN_INFO(441, 0, 0, 0, 0)
#define SC9861_RFCTL12            SPRD_PIN_INFO(443, 0, 0, 0, 0)
#define SC9861_RFCTL13            SPRD_PIN_INFO(445, 0, 0, 0, 0)
#define SC9861_RFCTL14            SPRD_PIN_INFO(447, 0, 0, 0, 0)
#define SC9861_RFCTL15            SPRD_PIN_INFO(449, 0, 0, 0, 0)
#define SC9861_RFCTL16            SPRD_PIN_INFO(451, 0, 0, 0, 0)
#define SC9861_RFCTL17            SPRD_PIN_INFO(453, 0, 0, 0, 0)
#define SC9861_RFCTL18            SPRD_PIN_INFO(455, 0, 0, 0, 0)
#define SC9861_RFCTL19            SPRD_PIN_INFO(457, 0, 0, 0, 0)
#define SC9861_RFCTL20            SPRD_PIN_INFO(459, 0, 0, 0, 0)
#define SC9861_RFCTL21            SPRD_PIN_INFO(461, 0, 0, 0, 0)
#define SC9861_RFCTL22            SPRD_PIN_INFO(463, 0, 0, 0, 0)
#define SC9861_RFCTL23            SPRD_PIN_INFO(465, 0, 0, 0, 0)
#define SC9861_RFCTL24            SPRD_PIN_INFO(467, 0, 0, 0, 0)
#define SC9861_RFCTL25            SPRD_PIN_INFO(469, 0, 0, 0, 0)

/* registers definitions for controller CTL_PIN_MISC */
#define SC9861_RFCTL32_MISC             SPRD_PIN_INFO(100, 2, 0, 0, 0)
#define SC9861_RFCTL33_MISC             SPRD_PIN_INFO(102, 2, 0, 0, 0)
#define SC9861_RFCTL34_MISC             SPRD_PIN_INFO(104, 2, 0, 0, 0)
#define SC9861_RFCTL35_MISC             SPRD_PIN_INFO(106, 2, 0, 0, 0)
#define SC9861_RFCTL36_MISC             SPRD_PIN_INFO(108, 2, 0, 0, 0)
#define SC9861_RFCTL37_MISC             SPRD_PIN_INFO(110, 2, 0, 0, 0)
#define SC9861_SPI0_CSN_MISC            SPRD_PIN_INFO(112, 2, 0, 0, 0)
#define SC9861_SPI0_DO_MISC             SPRD_PIN_INFO(114, 2, 0, 0, 0)
#define SC9861_SPI0_DI_MISC             SPRD_PIN_INFO(116, 2, 0, 0, 0)
#define SC9861_SPI0_CLK_MISC            SPRD_PIN_INFO(118, 2, 0, 0, 0)
#define SC9861_USB30_CC_SWITCH_MISC     SPRD_PIN_INFO(120, 2, 0, 0, 0)
#define SC9861_U1TXD_MISC               SPRD_PIN_INFO(122, 2, 0, 0, 0)
#define SC9861_U1RXD_MISC               SPRD_PIN_INFO(124, 2, 0, 0, 0)
#define SC9861_IIS1DI_MISC              SPRD_PIN_INFO(126, 2, 0, 0, 0)
#define SC9861_IIS1DO_MISC              SPRD_PIN_INFO(128, 2, 0, 0, 0)
#define SC9861_IIS1CLK_MISC             SPRD_PIN_INFO(130, 2, 0, 0, 0)
#define SC9861_IIS1LRCK_MISC            SPRD_PIN_INFO(132, 2, 0, 0, 0)
#define SC9861_U2TXD_MISC               SPRD_PIN_INFO(134, 2, 0, 0, 0)
#define SC9861_U2RXD_MISC               SPRD_PIN_INFO(136, 2, 0, 0, 0)
#define SC9861_IIS3CLK_MISC             SPRD_PIN_INFO(138, 2, 0, 0, 0)
#define SC9861_IIS3LRCK_MISC            SPRD_PIN_INFO(140, 2, 0, 0, 0)
#define SC9861_IIS3DI_MISC              SPRD_PIN_INFO(142, 2, 0, 0, 0)
#define SC9861_IIS3DO_MISC              SPRD_PIN_INFO(144, 2, 0, 0, 0)
#define SC9861_SD2_CMD_MISC             SPRD_PIN_INFO(146, 2, 0, 0, 0)
#define SC9861_SD2_D0_MISC              SPRD_PIN_INFO(148, 2, 0, 0, 0)
#define SC9861_SD2_D1_MISC              SPRD_PIN_INFO(150, 2, 0, 0, 0)
#define SC9861_SD2_CLK_MISC             SPRD_PIN_INFO(152, 2, 0, 0, 0)
#define SC9861_SD2_D2_MISC              SPRD_PIN_INFO(154, 2, 0, 0, 0)
#define SC9861_SD2_D3_MISC              SPRD_PIN_INFO(156, 2, 0, 0, 0)
#define SC9861_SD2_DUMY_MISC            SPRD_PIN_INFO(158, 2, 0, 0, 0)
#define SC9861_U4TXD_MISC               SPRD_PIN_INFO(160, 2, 0, 0, 0)
#define SC9861_U4RXD_MISC               SPRD_PIN_INFO(162, 2, 0, 0, 0)
#define SC9861_DCDC_ARM_EN1_MISC        SPRD_PIN_INFO(164, 2, 0, 0, 0)
#define SC9861_SENSOR_HUB_ACTION_MISC   SPRD_PIN_INFO(166, 2, 0, 0, 0)
#define SC9861_PTEST_MISC               SPRD_PIN_INFO(168, 2, 0, 0, 0)
#define SC9861_ANA_INT_MISC             SPRD_PIN_INFO(170, 2, 0, 0, 0)
#define SC9861_EXT_RST_B_MISC           SPRD_PIN_INFO(172, 2, 0, 0, 0)
#define SC9861_AUD_SCLK_MISC            SPRD_PIN_INFO(174, 2, 0, 0, 0)
#define SC9861_DCDC_ARM_EN0_MISC        SPRD_PIN_INFO(176, 2, 0, 0, 0)
#define SC9861_CLK_32K_MISC             SPRD_PIN_INFO(178, 2, 0, 0, 0)
#define SC9861_CHIP_SLEEP_MISC          SPRD_PIN_INFO(180, 2, 0, 0, 0)
#define SC9861_AUD_ADD0_MISC            SPRD_PIN_INFO(182, 2, 0, 0, 0)
#define SC9861_AUD_DAD0_MISC            SPRD_PIN_INFO(184, 2, 0, 0, 0)
#define SC9861_AUD_ADD1_MISC            SPRD_PIN_INFO(186, 2, 0, 0, 0)
#define SC9861_AUD_DAD1_MISC            SPRD_PIN_INFO(188, 2, 0, 0, 0)
#define SC9861_AUD_SYNC_MISC            SPRD_PIN_INFO(190, 2, 0, 0, 0)
#define SC9861_ADI_SCLK_MISC            SPRD_PIN_INFO(192, 2, 0, 0, 0)
#define SC9861_ADI_D_MISC               SPRD_PIN_INFO(194, 2, 0, 0, 0)
#define SC9861_MTCK_ARM_MISC            SPRD_PIN_INFO(196, 2, 0, 0, 0)
#define SC9861_MTMS_ARM_MISC            SPRD_PIN_INFO(198, 2, 0, 0, 0)
#define SC9861_MTRST_N_ARM_MISC         SPRD_PIN_INFO(200, 2, 0, 0, 0)
#define SC9861_MTDO_ARM_MISC            SPRD_PIN_INFO(202, 2, 0, 0, 0)
#define SC9861_MTDI_ARM_MISC            SPRD_PIN_INFO(204, 2, 0, 0, 0)
#define SC9861_EXTINT16_MISC            SPRD_PIN_INFO(206, 2, 0, 0, 0)
#define SC9861_KEYOUT0_MISC             SPRD_PIN_INFO(208, 2, 0, 0, 0)
#define SC9861_KEYOUT1_MISC             SPRD_PIN_INFO(210, 2, 0, 0, 0)
#define SC9861_KEYOUT2_MISC             SPRD_PIN_INFO(212, 2, 0, 0, 0)
#define SC9861_KEYIN0_MISC              SPRD_PIN_INFO(214, 2, 0, 0, 0)
#define SC9861_KEYIN1_MISC              SPRD_PIN_INFO(216, 2, 0, 0, 0)
#define SC9861_KEYIN2_MISC              SPRD_PIN_INFO(218, 2, 0, 0, 0)
#define SC9861_SD0_D0_MISC              SPRD_PIN_INFO(220, 2, 0, 0, 0)
#define SC9861_SD0_D1_MISC              SPRD_PIN_INFO(222, 2, 0, 0, 0)
#define SC9861_SD0_CLK0_MISC            SPRD_PIN_INFO(224, 2, 0, 0, 0)
#define SC9861_SD0_CMD_MISC             SPRD_PIN_INFO(226, 2, 0, 0, 0)
#define SC9861_SD0_D2_MISC              SPRD_PIN_INFO(228, 2, 0, 0, 0)
#define SC9861_SD0_D3_MISC              SPRD_PIN_INFO(230, 2, 0, 0, 0)
#define SC9861_SD0_DUMY_MISC            SPRD_PIN_INFO(232, 2, 0, 0, 0)
#define SC9861_SIMCLK2_MISC             SPRD_PIN_INFO(234, 2, 0, 0, 0)
#define SC9861_SIMDA2_MISC              SPRD_PIN_INFO(236, 2, 0, 0, 0)
#define SC9861_SIMRST2_MISC             SPRD_PIN_INFO(238, 2, 0, 0, 0)
#define SC9861_SIMCLK1_MISC             SPRD_PIN_INFO(240, 2, 0, 0, 0)
#define SC9861_SIMDA1_MISC              SPRD_PIN_INFO(242, 2, 0, 0, 0)
#define SC9861_SIMRST1_MISC             SPRD_PIN_INFO(244, 2, 0, 0, 0)
#define SC9861_SIMCLK0_MISC             SPRD_PIN_INFO(246, 2, 0, 0, 0)
#define SC9861_SIMDA0_MISC              SPRD_PIN_INFO(248, 2, 0, 0, 0)
#define SC9861_SIMRST0_MISC             SPRD_PIN_INFO(250, 2, 0, 0, 0)
#define SC9861_EMMC_CLK_MISC            SPRD_PIN_INFO(252, 2, 0, 0, 0)
#define SC9861_EMMC_RSTB_MISC           SPRD_PIN_INFO(254, 2, 0, 0, 0)
#define SC9861_EMMC_CMD_MISC            SPRD_PIN_INFO(256, 2, 0, 0, 0)
#define SC9861_EMMC_D0_MISC             SPRD_PIN_INFO(258, 2, 0, 0, 0)
#define SC9861_EMMC_D1_MISC             SPRD_PIN_INFO(260, 2, 0, 0, 0)
#define SC9861_EMMC_D2_MISC             SPRD_PIN_INFO(262, 2, 0, 0, 0)
#define SC9861_EMMC_D3_MISC             SPRD_PIN_INFO(264, 2, 0, 0, 0)
#define SC9861_EMMC_D4_MISC             SPRD_PIN_INFO(266, 2, 0, 0, 0)
#define SC9861_EMMC_D5_MISC             SPRD_PIN_INFO(268, 2, 0, 0, 0)
#define SC9861_EMMC_D6_MISC             SPRD_PIN_INFO(270, 2, 0, 0, 0)
#define SC9861_EMMC_D7_MISC             SPRD_PIN_INFO(272, 2, 0, 0, 0)
#define SC9861_EMMC_STROBE_MISC         SPRD_PIN_INFO(274, 2, 0, 0, 0)
#define SC9861_EMMC_DUMY_MISC           SPRD_PIN_INFO(276, 2, 0, 0, 0)
#define SC9861_SD1_CMD_MISC             SPRD_PIN_INFO(278, 2, 0, 0, 0)
#define SC9861_SD1_D0_MISC              SPRD_PIN_INFO(280, 2, 0, 0, 0)
#define SC9861_SD1_D1_MISC              SPRD_PIN_INFO(282, 2, 0, 0, 0)
#define SC9861_SD1_CLK_MISC             SPRD_PIN_INFO(284, 2, 0, 0, 0)
#define SC9861_SD1_D2_MISC              SPRD_PIN_INFO(286, 2, 0, 0, 0)
#define SC9861_SD1_D3_MISC              SPRD_PIN_INFO(288, 2, 0, 0, 0)
#define SC9861_SD1_DUMY_MISC            SPRD_PIN_INFO(290, 2, 0, 0, 0)
#define SC9861_IIS0DI_MISC              SPRD_PIN_INFO(292, 2, 0, 0, 0)
#define SC9861_IIS0DO_MISC              SPRD_PIN_INFO(294, 2, 0, 0, 0)
#define SC9861_IIS0CLK_MISC             SPRD_PIN_INFO(296, 2, 0, 0, 0)
#define SC9861_IIS0LRCK_MISC            SPRD_PIN_INFO(298, 2, 0, 0, 0)
#define SC9861_U3TXD_MISC               SPRD_PIN_INFO(300, 2, 0, 0, 0)
#define SC9861_U3RXD_MISC               SPRD_PIN_INFO(302, 2, 0, 0, 0)
#define SC9861_U3CTS_MISC               SPRD_PIN_INFO(304, 2, 0, 0, 0)
#define SC9861_U3RTS_MISC               SPRD_PIN_INFO(306, 2, 0, 0, 0)
#define SC9861_U0TXD_MISC               SPRD_PIN_INFO(308, 2, 0, 0, 0)
#define SC9861_U0RXD_MISC               SPRD_PIN_INFO(310, 2, 0, 0, 0)
#define SC9861_U0CTS_MISC               SPRD_PIN_INFO(312, 2, 0, 0, 0)
#define SC9861_U0RTS_MISC               SPRD_PIN_INFO(314, 2, 0, 0, 0)
#define SC9861_CLK_AUX0_MISC            SPRD_PIN_INFO(316, 2, 0, 0, 0)
#define SC9861_RFCTL39_MISC             SPRD_PIN_INFO(318, 2, 0, 0, 0)
#define SC9861_RFCTL38_MISC             SPRD_PIN_INFO(320, 2, 0, 0, 0)
#define SC9861_WIFI_COEXIST_MISC        SPRD_PIN_INFO(322, 2, 0, 0, 0)
#define SC9861_BEIDOU_COEXIST_MISC      SPRD_PIN_INFO(324, 2, 0, 0, 0)
#define SC9861_EXTINT12_MISC            SPRD_PIN_INFO(326, 2, 0, 0, 0)
#define SC9861_EXTINT11_MISC            SPRD_PIN_INFO(328, 2, 0, 0, 0)
#define SC9861_EXTINT10_MISC            SPRD_PIN_INFO(330, 2, 0, 0, 0)
#define SC9861_EXTINT9_MISC             SPRD_PIN_INFO(332, 2, 0, 0, 0)
#define SC9861_EXTINT8_MISC             SPRD_PIN_INFO(334, 2, 0, 0, 0)
#define SC9861_EXTINT7_MISC             SPRD_PIN_INFO(336, 2, 0, 0, 0)
#define SC9861_EXTINT6_MISC             SPRD_PIN_INFO(338, 2, 0, 0, 0)
#define SC9861_SDA1_MISC                SPRD_PIN_INFO(340, 2, 0, 0, 0)
#define SC9861_SCL1_MISC                SPRD_PIN_INFO(342, 2, 0, 0, 0)
#define SC9861_EXTINT1_MISC             SPRD_PIN_INFO(344, 2, 0, 0, 0)
#define SC9861_EXTINT0_MISC             SPRD_PIN_INFO(346, 2, 0, 0, 0)
#define SC9861_EXTINT5_MISC             SPRD_PIN_INFO(348, 2, 0, 0, 0)
#define SC9861_DSI_TE_MISC              SPRD_PIN_INFO(350, 2, 0, 0, 0)
#define SC9861_LCM_RSTN_MISC            SPRD_PIN_INFO(352, 2, 0, 0, 0)
#define SC9861_PWMA_MISC                SPRD_PIN_INFO(354, 2, 0, 0, 0)
#define SC9861_SCL2_MISC                SPRD_PIN_INFO(356, 2, 0, 0, 0)
#define SC9861_SDA2_MISC                SPRD_PIN_INFO(358, 2, 0, 0, 0)
#define SC9861_CMPD1_MISC               SPRD_PIN_INFO(360, 2, 0, 0, 0)
#define SC9861_CMPD0_MISC               SPRD_PIN_INFO(362, 2, 0, 0, 0)
#define SC9861_CMRST1_MISC              SPRD_PIN_INFO(364, 2, 0, 0, 0)
#define SC9861_CMRST0_MISC              SPRD_PIN_INFO(366, 2, 0, 0, 0)
#define SC9861_CMMCLK1_MISC             SPRD_PIN_INFO(368, 2, 0, 0, 0)
#define SC9861_CMMCLK_MISC              SPRD_PIN_INFO(370, 2, 0, 0, 0)
#define SC9861_RFSEN0_MISC              SPRD_PIN_INFO(372, 2, 0, 0, 0)
#define SC9861_RFSCK0_MISC              SPRD_PIN_INFO(374, 2, 0, 0, 0)
#define SC9861_RFSDA0_MISC              SPRD_PIN_INFO(376, 2, 0, 0, 0)
#define SC9861_RFSEN1_MISC              SPRD_PIN_INFO(378, 2, 0, 0, 0)
#define SC9861_RFSCK1_MISC              SPRD_PIN_INFO(380, 2, 0, 0, 0)
#define SC9861_RFSDA1_MISC              SPRD_PIN_INFO(382, 2, 0, 0, 0)
#define SC9861_RFCTL27_MISC             SPRD_PIN_INFO(384, 2, 0, 0, 0)
#define SC9861_RFCTL26_MISC             SPRD_PIN_INFO(386, 2, 0, 0, 0)
#define SC9861_RFCTL31_MISC             SPRD_PIN_INFO(388, 2, 0, 0, 0)
#define SC9861_RFCTL30_MISC             SPRD_PIN_INFO(390, 2, 0, 0, 0)
#define SC9861_RF_LVDS0_ADC_ON_MISC     SPRD_PIN_INFO(392, 2, 0, 0, 0)
#define SC9861_RF_LVDS0_DAC_ON_MISC     SPRD_PIN_INFO(394, 2, 0, 0, 0)
#define SC9861_RF_LVDS1_ADC_ON_MISC     SPRD_PIN_INFO(396, 2, 0, 0, 0)
#define SC9861_RF_LVDS1_DAC_ON_MISC     SPRD_PIN_INFO(398, 2, 0, 0, 0)
#define SC9861_SDA4_MISC                SPRD_PIN_INFO(400, 2, 0, 0, 0)
#define SC9861_SCL4_MISC                SPRD_PIN_INFO(402, 2, 0, 0, 0)
#define SC9861_SDA0_MISC                SPRD_PIN_INFO(404, 2, 0, 0, 0)
#define SC9861_SCL0_MISC                SPRD_PIN_INFO(406, 2, 0, 0, 0)
#define SC9861_RFCTL29_MISC             SPRD_PIN_INFO(408, 2, 0, 0, 0)
#define SC9861_RFCTL28_MISC             SPRD_PIN_INFO(410, 2, 0, 0, 0)
#define SC9861_RFFE0_SDA0_MISC          SPRD_PIN_INFO(412, 2, 0, 0, 0)
#define SC9861_RFFE0_SCK0_MISC          SPRD_PIN_INFO(414, 2, 0, 0, 0)
#define SC9861_RFFE1_SDA0_MISC          SPRD_PIN_INFO(416, 2, 0, 0, 0)
#define SC9861_RFFE1_SCK0_MISC          SPRD_PIN_INFO(418, 2, 0, 0, 0)
#define SC9861_RFCTL0_MISC              SPRD_PIN_INFO(420, 2, 0, 0, 0)
#define SC9861_RFCTL1_MISC              SPRD_PIN_INFO(422, 2, 0, 0, 0)
#define SC9861_RFCTL2_MISC              SPRD_PIN_INFO(424, 2, 0, 0, 0)
#define SC9861_RFCTL3_MISC              SPRD_PIN_INFO(426, 2, 0, 0, 0)
#define SC9861_RFCTL4_MISC              SPRD_PIN_INFO(428, 2, 0, 0, 0)
#define SC9861_RFCTL5_MISC              SPRD_PIN_INFO(430, 2, 0, 0, 0)
#define SC9861_RFCTL6_MISC              SPRD_PIN_INFO(432, 2, 0, 0, 0)
#define SC9861_RFCTL7_MISC              SPRD_PIN_INFO(434, 2, 0, 0, 0)
#define SC9861_RFCTL8_MISC              SPRD_PIN_INFO(436, 2, 0, 0, 0)
#define SC9861_RFCTL9_MISC              SPRD_PIN_INFO(438, 2, 0, 0, 0)
#define SC9861_RFCTL10_MISC             SPRD_PIN_INFO(440, 2, 0, 0, 0)
#define SC9861_RFCTL11_MISC             SPRD_PIN_INFO(442, 2, 0, 0, 0)
#define SC9861_RFCTL12_MISC             SPRD_PIN_INFO(444, 2, 0, 0, 0)
#define SC9861_RFCTL13_MISC             SPRD_PIN_INFO(446, 2, 0, 0, 0)
#define SC9861_RFCTL14_MISC             SPRD_PIN_INFO(448, 2, 0, 0, 0)
#define SC9861_RFCTL15_MISC             SPRD_PIN_INFO(450, 2, 0, 0, 0)
#define SC9861_RFCTL16_MISC             SPRD_PIN_INFO(452, 2, 0, 0, 0)
#define SC9861_RFCTL17_MISC             SPRD_PIN_INFO(454, 2, 0, 0, 0)
#define SC9861_RFCTL18_MISC             SPRD_PIN_INFO(456, 2, 0, 0, 0)
#define SC9861_RFCTL19_MISC             SPRD_PIN_INFO(458, 2, 0, 0, 0)
#define SC9861_RFCTL20_MISC             SPRD_PIN_INFO(460, 2, 0, 0, 0)
#define SC9861_RFCTL21_MISC             SPRD_PIN_INFO(462, 2, 0, 0, 0)
#define SC9861_RFCTL22_MISC             SPRD_PIN_INFO(464, 2, 0, 0, 0)
#define SC9861_RFCTL23_MISC             SPRD_PIN_INFO(466, 2, 0, 0, 0)
#define SC9861_RFCTL24_MISC             SPRD_PIN_INFO(468, 2, 0, 0, 0)
#define SC9861_RFCTL25_MISC             SPRD_PIN_INFO(470, 2, 0, 0, 0)
