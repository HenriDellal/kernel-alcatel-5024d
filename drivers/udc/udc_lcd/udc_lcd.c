/* drivers/video/udc/udc_lcd/udc_lcd.c
 *
 *
 * Copyright (C) 2010 Spreadtrum
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/udc.h>
//#include <../udc_base.h>
//#include "udc_lcd.h"
#include <dsi/mipi_dsi_api.h>
#include "sprd_dphy.h"
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>



static int32_t udc_lcd_init(udc_panel_device *panel_device);
static uint32_t udc_lcd_read_id(udc_panel_device *panel_device);

static int32_t udc_lcd_enter_sleep( udc_panel_device *panel_device);

static uint32_t udc_lcd_start(udc_panel_device *panel_device, udc_t key_id,uint16_t left, uint16_t top,uint16_t right,uint16_t bottom);
static int32_t udc_lcd_check_esd(udc_panel_device *panel_device);



static udc_panel_ops lcd_udc_lcd_operations = {
	.init = udc_lcd_init,
	.sleep_in = udc_lcd_enter_sleep,
	.read_id = udc_lcd_read_id,
	.esd_check = udc_lcd_check_esd,
};


static udc_item_s g_udc_lcd_item[] = {
	{LCD_FMARK               ,   0,   0},	
	{LCD_NAME               ,   0,   0},
	{LCD_ID               ,   0,   0},
	{LCD_WIDTH            ,   0,   0},
	{LCD_HIGHT            ,   0,   0},
	{LCD_MODE             ,   0,   0},
	{LCD_DIRECTION        ,   0,   0},
	{LCD_BUS_MODE         ,   0,   0},
	{LCD_BUS_WIDTH        ,   0,   0},
	{LCD_TIMING0          ,   0,   0},
	{LCD_TIMING1          ,   0,   0},
	{LCD_READ_ID          ,   0,   0},
	{LCD_INIT_PARA        ,   0,   0},
	{LCD_SET_WINDOW       ,   0,   0},
	{LCD_INVALIDATE_RECT  ,   0,   0},
	{LCD_DIR_NORMAL       ,   0,   0},
	{LCD_DIR_ROT90        ,   0,   0},
	{LCD_DIR_ROT180       ,   0,   0},
	{LCD_DIR_ROT270       ,   0,   0},
	{LCD_DIR_MIRH         ,   0,   0},
	{LCD_DIR_MIRV         ,   0,   0},
	{LCD_DIR_MIRHV        ,   0,   0},
	{LCD_ENTER_SLEEP      ,   0,   0},
	{LCD_EXIST_SLEEP      ,   0,   0},
	{LCD_WORK_MODE       ,   0,   0},
	{LCD_LAN_NUM         ,   0,   0},
	{LCD_PHY_FEQ         ,   0,   0},
	{LCD_H_SYNC_POL      ,   0,   0},
	{LCD_V_SYNC_POL      ,   0,   0},
	{LCD_DE_POL          ,   0,   0},
	{LCD_TE_POL          ,   0,   0},
	{LCD_COLOR_MODE_POL  ,   0,   0},
	{LCD_SHUT_DOWN_POL   ,   0,   0},
	{LCD_POWER_MODE      ,   0,   0},
	{LCD_READ_POWERMODE   ,   0,   0},
	{LCD_SPEED_MODE   ,   0,   0},
	{LCD_FPS   ,   0,   0},	
	{LCD_SUSPEND_MODE   ,   0,   0},	
	{LCD_DSC_COMPRESSION   ,   0,   0},	
	{LCD_POWER_ON_SEQUENCE   ,   0,	0}, 
	{LCD_POWER_OFF_SEQUENCE   ,   0,	0}, 
	{LCD_WIDTH_MM   ,   0,   0},
	{LCD_HEIGHT_MM   ,   0,   0},	
	{LCD_SIMU_WIDTH   ,   0,   0},	
	{LCD_SIMU_HEIGHT   ,   0,   0},	
	{LCD_BURST_MODE   ,   0,	0}, 
	{0XFFFF               ,   0,   0}
};


static udc_lcd g_udc_lcd;


#ifdef CONFIG_UDC_LCD_MIPI

static int32_t udc_lcd_check_esd(udc_panel_device *panel_device)
{

	struct sprd_dsi *dsi = dev_get_drvdata(panel_device->intf);
	struct panel_info *panel = panel_device->panel;
	udc_lcd* lcd = udc_get_lcd(SEC_LCD0);
	uint32_t power_mode;

	mipi_dsi_lp_cmd_enable(dsi, false);
	mipi_dsi_set_max_return_size(dsi, panel->esd_read_count);

	lcd->power_mode=lcd->panel_device->panel->esd_return_code[0];

	if(lcd->power_mode==0)
	{ 
		//UDC_LCD_TRACE("[udc_lcd_check_esd] jinq:The power mode if defined, you can't is 0\n");
		return 0;
	}
	power_mode = udc_lcd_start(panel_device,LCD_READ_POWERMODE,0,0,0,0);
	if(power_mode == lcd->power_mode){
		//printk("check_esd OK!\n");
		return 0;
	}else{
		printk("check_esd fail!(0x%x)\n", power_mode);
		return -ENODATA;
	}


}




static int udc_lcd_config_lcm_timing_mipi(udc_t key_id, struct display_timing *timing)
{
	unsigned short mipi_timing[6];
	if (udc_lcd_get_value(key_id, mipi_timing, 6) )
	{
		timing->hfront_porch.typ = timing->hfront_porch.min =timing->hfront_porch.max =mipi_timing[0];
		timing->hback_porch.typ = timing->hback_porch.min =timing->hback_porch.max =mipi_timing[1];
		timing->hsync_len.typ = timing->hsync_len.min = timing->hsync_len.max = mipi_timing[2];
		timing->vfront_porch.typ =timing->vfront_porch.min =timing->vfront_porch.max = mipi_timing[3];
		timing->vback_porch.typ =timing->vback_porch.min =timing->vback_porch.max = mipi_timing[4];
		timing->vsync_len.typ =timing->vsync_len.min =timing->vsync_len.max = mipi_timing[5];
		UDC_LCD_TRACE("id = %d hfp %d, hbp %d, hsync %d, vfp %d, vbp %d, vsync %d\n", key_id,
			timing->hfront_porch.typ, timing->hback_porch.typ,	timing->hsync_len.typ, timing->vfront_porch.typ, timing->vback_porch.typ, timing->vsync_len.typ);
		return 1;
	}
	return 0;
}


static uint32_t udc_lcd_mipi_do(udc_panel_device *panel_device, unsigned short *value, unsigned short value_count,
		uint16_t left, uint16_t top,uint16_t right,uint16_t bottom)
{


	#define MAX_DATA   100

	uint32_t i=0, j=0;
	uint8_t read_data[MAX_DATA];
	uint8_t data[MAX_DATA];
	uint16_t cmd,len;
	uint8_t datatype;
	unsigned int *p;

	unsigned int items;
	struct gpio_timing *timing;

	struct sprd_dsi *dsi_ctx = dev_get_drvdata(panel_device->intf);


	UDC_LCD_TRACE("%s: value = 0x%x, value_count = %d\n", __func__, *value, value_count);
	
  for (i=0; i<value_count;) 
	{
		cmd = value[i];
	
		if(UDC_LCD_POWER_ON == cmd)
		{
			items = panel_device->panel->pwr_on_seq.items;
			timing = panel_device->panel->pwr_on_seq.timing;
			items= value[i+1]/3;
			panel_device->panel->pwr_on_seq.items=items;
			p = kzalloc(value[i+1]*sizeof(unsigned int), GFP_KERNEL);
			if (!p) {
				printk("error udc_lcd_mipi_do 1 alloc interface fail\n");
			}

			for(j = 0; j < value[i+1]; j++)
			{
				p[j] = (uint8_t)value[i+2+j];
				UDC_LCD_TRACE("UDC_LCD_POWER_ON  data[%d]= %d\n",j,p[j]);
			}
			(panel_device->panel->pwr_on_seq.timing)=(struct gpio_timing *)p;
			i =value[i+1]+2;
			//kfree(p);

		}
		if(UDC_LCD_POWER_OFF == cmd)
		{
			items = panel_device->panel->pwr_off_seq.items;
			timing = panel_device->panel->pwr_off_seq.timing;
			items= value[i+1]/3;
			panel_device->panel->pwr_off_seq.items=items;
			p = kzalloc(value[i+1]*sizeof(unsigned int), GFP_KERNEL);
			if (!p) {
				printk("error udc_lcd_mipi_do 2 alloc interface fail\n");
			}
			
			for(j = 0; j < value[i+1]; j++)
			{
				p[j] = (uint8_t)value[i+2+j];
				UDC_LCD_TRACE("UDC_LCD_POWER_OFF  data[%d]= %d\n",j,p[j]);
			}
			(panel_device->panel->pwr_off_seq.timing)=(struct gpio_timing *)p;
			i =value[i+1]+2;
			//kfree(p);
		}
	
		if(UDC_LCD_MIPI_SET_LP_MODE == cmd)
		{
		//	ops->mipi_set_lp_mode(dsi_ctx);
			i += 1;
		}
		if(UDC_LCD_MIPI_SET_DATA_LP_MODE == cmd)
		{
		//	ops->mipi_set_data_lp_mode(dsi_ctx);
			i += 1;
		}
		if(UDC_LCD_MIPI_SET_HS_MODE == cmd)
		{
		//	ops->mipi_set_hs_mode(dsi_ctx);
			i += 1;
		}
		if(UDC_LCD_MIPI_SET_DATA_HS_MODE == cmd)
		{
			//ops->mipi_set_data_hs_mode(dsi_ctx);
			i += 1;
		}
		if(UDC_LCD_MIPI_SET_CMD_MODE == cmd)
		{
			//ops->mipi_set_cmd_mode(dsi_ctx);
			i += 1;
		}	
			
		if(UDC_LCD_MIPI_EOTP_SET == cmd)
		{
		    UDC_LCD_TRACE("param1 = %d,param2 = %d\n",value[i+1],value[i+2]);
		//	mipi_eotp_set(dsi_ctx,value[i+1],value[i+2]);
			i += 3;
		}		
	
		if(UDC_LCD_RETURN_ID == cmd)
		{	 
			read_data[0] = value[i+1];
			UDC_LCD_TRACE("return id 0x%X\n", read_data[0]);
		}

		if(UDC_LCD_MIPI_FORCE_WRITE ==cmd)
		{
			len = value[i+2];
			if(len > 2)
			{	
				len=value[i+3];
				datatype = (uint8_t)value[i+1];
				for(j = 0; j < len; j++)
				{
					data[j] = (uint8_t)value[i+5+j];
				}
				UDC_LCD_TRACE("data_type = 0x%x,len = %d,param = 0x%x\n",datatype,len,data[0]);
				mipi_dsi_force_write(dsi_ctx,datatype, data, len);//type,data,len
				i += len+5;
			}
			else	
			{
				datatype = (uint8_t)value[i+1];
				for(j = 0; j < len; j++)
				{
					data[j] = (uint8_t)value[i+3+j];
				}
				UDC_LCD_TRACE("data_type = 0x%x,len = %d,param = 0x%x\n",datatype,len,data[0]);
				mipi_dsi_force_write(dsi_ctx,datatype, data, len);//type,data,len
				i += len+3;
			}
		}

		if(UDC_LCD_MIPI_GEN_WRITE ==cmd)
		{
			if(value[i+1] >2)
			{
				len = value[i+2];
				for(j = 0; j < len; j++)
				{
					data[j] = (uint8_t)value[i+4+j];
		
				}
				mipi_dsi_gen_write(dsi_ctx,data, len);//data,len
				i += len+4;
			
			}
			else
			{
				len = value[i+1];
				for(j = 0; j < len; j++)
				{
					data[j] = (uint8_t)value[i+2+j];
				
				}
				mipi_dsi_gen_write(dsi_ctx,data, len);//data,len
				i += len+2;
			}
		
		}
		if(UDC_LCD_MIPI_FORCE_READ ==cmd)
		{ 
			len = value[i+2];
			data[0] = (uint8_t)value[i+1];
			UDC_LCD_TRACE("addr = 0x%x,len = %d\n",data[0],len);
		//	mipi_force_read(dsi_ctx,data[0], len,read_data);//addr,len,buf
		
			mipi_dsi_dcs_read(dsi_ctx, data[0], read_data,1);
			read_data[0] = read_data[value[i+3]];   //?DD?y?Y
			i += 4;
		}
		if(UDC_LCD_MIPI_GEN_READ ==cmd)
		{
				//mipi_gen_read(data, len));
		}
		if(UDC_LCD_DELAY_MS == cmd)
		{
			msleep(value[i+1]);
			i += 2;
		}
		udelay(30);
	}
	UDC_LCD_TRACE("read_data = 0x%x\n",read_data[0]);


	return read_data[0];


}

#endif



static uint32_t udc_lcd_start(udc_panel_device *panel_device, udc_t key_id,
	                                          uint16_t left, uint16_t top,uint16_t right,uint16_t bottom)
{
	udc_t *value;
	uint32_t read_param = 0;
	unsigned short value_count;
    udc_lcd* lcd = udc_get_lcd(SEC_LCD0);  
 // UDC_LCD_TRACE("%s: key_id = %d, pannel_type = %d \n", __func__, key_id, pannel_type);   
	if (key_id >= CONFIG_MAX_ID) {
		UDC_LCD_TRACE("%s invalid sub name id %d\n", __func__, key_id);
		return (uint32_t)-EINVAL;
	}        
	value =   lcd->item[key_id].value;
	value_count = lcd->item[key_id].value_count;

	if ((value != NULL) && (value_count>0 ))
	{
		UDC_LCD_TRACE("%s: value = 0x%x, value_count = %d\n", __func__, *value, value_count);
	#ifdef CONFIG_UDC_LCD_MIPI
		read_param = udc_lcd_mipi_do(panel_device, value, value_count, left, top, right, bottom);
	#endif			
	}

	return read_param;
}


static int32_t udc_lcd_init(udc_panel_device *panel_device)
{
	//uint16_t lcd_speed_mode =0;
	struct sprd_dsi *dsi = dev_get_drvdata(panel_device->intf);
	struct device *dev = dev_get_next(&dsi->dev);
	struct sprd_dphy *dphy = dev_get_drvdata(dev);
	struct panel_info *panel = panel_device->panel;
	UDC_LCD_TRACE("%s\n", __func__);


  	mipi_dsi_lp_cmd_enable(dsi, true);
 	udc_lcd_start(panel_device,LCD_INIT_PARA,0,0,0,0);
	mipi_dsi_set_work_mode(dsi, panel->work_mode);
	mipi_dsi_state_reset(dsi);
	mipi_dphy_hs_clk_en(dphy, true);

	return 0;
}

int udc_get_power_param(udc_panel_device  *device)
{

	if(device)
	{

		udc_lcd_start(device,LCD_POWER_ON_SEQUENCE,0,0,0,0);
		udc_lcd_start(device,LCD_POWER_OFF_SEQUENCE,0,0,0,0);
	}
	return 0;
}
EXPORT_SYMBOL(udc_get_power_param);



static int32_t udc_lcd_enter_sleep(udc_panel_device *panel_device)
{

	struct sprd_dsi *dsi = dev_get_drvdata(panel_device->intf);
	UDC_LCD_TRACE("%s\n", __func__);
	mipi_dsi_set_work_mode(dsi, SPRD_MIPI_MODE_CMD);
	mipi_dsi_lp_cmd_enable(dsi, true);

	udc_lcd_start(panel_device,LCD_ENTER_SLEEP,0,0,0,0);

	return 0;
}

static uint32_t udc_lcd_read_id(udc_panel_device*panel_device)
{
	udc_lcd	*lcd = udc_get_lcd(SEC_LCD0);
	int	j = 0;
	int lcm_id, lcm_cfg_id;
	UDC_LCD_TRACE("%s\n", __func__);
	lcm_cfg_id = lcd->panel_device->panel->id_val[0];
	UDC_LCD_TRACE("%s lcm_cfg_id=0x%x\n", __func__, lcm_cfg_id);
	for(j = 0; j < 4; j++){
	lcm_id = udc_lcd_start(panel_device,LCD_READ_ID,0,0,0,0);
	if (lcm_id <= 0)
		lcm_id = UDC_LCM_ID;
	
	UDC_LCD_TRACE("%s 0x%x\n", __func__, lcm_id);
	if (lcm_id == lcm_cfg_id)
		break;
	}
	return lcm_id;
}



udc_t udc_lcd_get_value(udc_t key_id, udc_t* value, udc_t value_count)
{
	udc_lcd* lcd = &g_udc_lcd;

	return udc_get_item_value(lcd->item, key_id, value, value_count);
}

udc_lcd* udc_get_lcd(udc_t section_id)
{
	return &g_udc_lcd;
}



int udc_lcd_config_panel(udc_lcd* lcd, udc_t section_id)
{
       udc_t ret;
	udc_t param;
//	udc_t pannel_type;
  	udc_panel_device  *panel_device = lcd->panel_device;
 	udc_panel_info    *panel_info = lcd->panel_device->panel;
	
//	panel_info = kzalloc(sizeof(struct panel_info), GFP_KERNEL);
//	if (panel_info == NULL)
//		return 0;

	ret = udc_match_item(lcd->udc, &lcd->current_section, lcd->item);
	UDC_LCD_TRACE("%s () line = %d, ret = %d\n", __func__, __LINE__, ret);
       if ( !ret )
	    return 0;	
	   
		
	//if ( udc_lcd_get_value(LCD_ID, &param, 1) )
	//	panel_info->lcd_id = param;

	if ( udc_lcd_get_value(LCD_WIDTH,  &param, 1) )
		panel_info->width = param;


       printk("udc_lcd_config_panel width=%d\n",param);
	if ( udc_lcd_get_value(LCD_HIGHT, &param, 1) )
		panel_info->height = param;

	//if ( udc_lcd_get_value(LCD_SUSPEND_MODE,  &param, 1) )
	//	panel_info->suspend_mode = param;
	
//	if ( udc_lcd_get_value(LCD_DIRECTION, &param, 1) )
	//	panel_info->direction= param;

	if (NULL != lcd->item[LCD_NAME].value)
		panel_info->lcd_name=(const char *)lcd->item[LCD_NAME].value;

	UDC_LCD_TRACE("lcd_name: %s\n", panel_info->lcd_name);
	UDC_LCD_TRACE("lcd_width %d\n", panel_info->width);
	UDC_LCD_TRACE("lcd_hight %d\n", panel_info->height);


	if ( udc_lcd_get_value(LCD_MODE, &param, 1) )
	{
		panel_info->type= param;
	}
   UDC_LCD_TRACE("lcd_type: %d\n", panel_info->type);

#ifdef CONFIG_UDC_LCD_MIPI
	if (panel_info->type == SPRD_PANEL_TYPE_MIPI) 
	{
          
	  if ( udc_lcd_get_value(LCD_WORK_MODE, &param, 1) )
		  panel_info->work_mode  = param;
	  UDC_LCD_TRACE("work_mode 0x%x\n", panel_info->work_mode);
	
	  if ( udc_lcd_get_value(LCD_BUS_WIDTH, &param, 1) )
	 	  panel_info->bpp = param;
	  UDC_LCD_TRACE("bpp 0x%x\n", panel_info->bpp);
	  
	  if ( udc_lcd_get_value(LCD_LAN_NUM, &param, 1) )
		  panel_info->lane_num = param;
	  UDC_LCD_TRACE("lan_number 0x%x\n", panel_info->lane_num);
	  
	  if ( udc_lcd_get_value(LCD_PHY_FEQ, &param, 1) )
		  panel_info->phy_freq = param*1000;
	  UDC_LCD_TRACE("phy_feq 0x%x\n", panel_info->phy_freq);
	
	  if ( udc_lcd_get_value(LCD_H_SYNC_POL, &param, 1) )
		  panel_info->h_sync_pol = param;
	  UDC_LCD_TRACE("h_sync_pol 0x%x\n", panel_info->h_sync_pol);   
	
	  if ( udc_lcd_get_value(LCD_V_SYNC_POL, &param, 1) )
		  panel_info->v_sync_pol = param;
	  UDC_LCD_TRACE("v_sync_pol 0x%x\n", panel_info->v_sync_pol);
	  
	  if ( udc_lcd_get_value(LCD_DE_POL, &param, 1) )
		  panel_info->de_pol = param;
	  UDC_LCD_TRACE("de_pol 0x%x\n", panel_info->de_pol);
	  
	  if ( udc_lcd_get_value(LCD_TE_POL, &param, 1) )
		  panel_info->te_pol = param;
	  UDC_LCD_TRACE("te_pol 0x%x\n", panel_info->te_pol);
	
	//  if ( udc_lcd_get_value(LCD_COLOR_MODE_POL, &param, 1) )
	//	  cfg->color_mode_pol = param;
	//  UDC_LCD_TRACE("color_mode_pol 0x%x\n", cfg->color_mode_pol);
	  
	//  if ( udc_lcd_get_value(LCD_SHUT_DOWN_POL, &param, 1) )
	//	  cfg->shut_down_pol = param;
	 // UDC_LCD_TRACE("shut_down_pol 0x%x\n", cfg->shut_down_pol);

	  if ( udc_lcd_get_value(LCD_DSC_COMPRESSION, &param, 1) )
		  panel_info->dsc_en = param;
	  UDC_LCD_TRACE("dsc_compression 0x%x\n", panel_info->dsc_en);

	  if ( udc_lcd_get_value(LCD_POWER_MODE, &param, 1) )
	  	{
		  panel_info->esd_return_code[0] = param;
		  if(param)
		  {
		  	if(param == 3)
		  	{
				panel_info->esd_check_en=3;
			}
			else
			{
				panel_info->esd_check_en=1;
			}
			panel_info->esd_timeout=1000;
		  }
	  	}
	
	  UDC_LCD_TRACE("power_mode 0x%x\n", lcd->power_mode);


	  udc_lcd_config_lcm_timing_mipi(LCD_TIMING0, &(panel_info->display_timing));
 	  panel_info->display_timing.pixelclock.typ=panel_info->display_timing.pixelclock.min=panel_info->display_timing.pixelclock.max=panel_info->phy_freq;
	  panel_info->display_timing.hactive.typ=panel_info->display_timing.hactive.min=panel_info->display_timing.hactive.max=panel_info->width;
	  panel_info->display_timing.vactive.typ=panel_info->display_timing.vactive.min=panel_info->display_timing.vactive.max=panel_info->height;
	  panel_info->display_timing.hactive.typ=panel_info->width;
	  panel_info->display_timing.vactive.typ=panel_info->height;

	//  panel_info->display_timing.flags |=  panel_info->display_timing.vactive.typ ? DISPLAY_FLAGS_VSYNC_HIGH :DISPLAY_FLAGS_VSYNC_LOW;
	 // panel_info->display_timing.flags |=  panel_info->display_timing.hactive.typ ? DISPLAY_FLAGS_HSYNC_HIGH :DISPLAY_FLAGS_HSYNC_LOW;
	  //panel_info->display_timing.flags |=  panel_info->de_pol ? DISPLAY_FLAGS_DE_HIGH : DISPLAY_FLAGS_DE_LOW;
	  //panel_info->display_timing.flags |=  panel_info->display_timing.pixelclock.typ ? DISPLAY_FLAGS_PIXDATA_POSEDGE : DISPLAY_FLAGS_PIXDATA_NEGEDGE;


	  // cfg->display_timing.flags =DISPLAY_FLAGS_VSYNC_LOW | DISPLAY_FLAGS_HSYNC_LOW | DISPLAY_FLAGS_DE_LOW | DISPLAY_FLAGS_PIXDATA_POSEDGE;

	if ( udc_lcd_get_value(LCD_WIDTH_MM, &param, 1) )
		panel_info->width_mm = param;
	else
		 panel_info->width_mm = 68;
	UDC_LCD_TRACE("width_mm %d\n", panel_info->width_mm);
	

	 if ( udc_lcd_get_value(LCD_HEIGHT_MM, &param, 1) )
		  panel_info->height_mm = param;
     else
		   panel_info->height_mm = 121;
	 UDC_LCD_TRACE("height_mm %d\n", panel_info->height_mm);

	if ( udc_lcd_get_value(LCD_SIMU_WIDTH, &param, 1) )
		panel_info->simu_width = param;
	else
		 panel_info->simu_width = 720;
	UDC_LCD_TRACE("simu_width %d\n", panel_info->simu_width);

	if ( udc_lcd_get_value(LCD_SIMU_HEIGHT, &param, 1) )
		panel_info->simu_height = param;
	else
		 panel_info->simu_height = 1280;
	UDC_LCD_TRACE("simu_height %d\n", panel_info->simu_height);


	if ( udc_lcd_get_value(LCD_BURST_MODE, &param, 1) )
		panel_info->burst_mode = param;
	else
		 panel_info->burst_mode = PANEL_VIDEO_BURST_MODE;
	UDC_LCD_TRACE("burst_mode %d\n", panel_info->burst_mode);


///
//	  UDC_LCD_TRACE(" cfg->display_timing.pixelclock.typ %d\n",  cfg->display_timing.pixelclock.typ);	
//	  UDC_LCD_TRACE(" cfg->display_timing.flags %d\n",  cfg->display_timing.flags);	

		if ( udc_lcd_get_value(LCD_FPS, &param, 1) )
			panel_info->fps = param;
		UDC_LCD_TRACE("lcd_fps %d\n", panel_info->fps);			
	}
#endif



	panel_device->ops=&lcd_udc_lcd_operations;




	return 1;

}



udc_lcd* udc_lcd_create(udc_t section_id, udc_panel_device  *panel_device)
{
    udc_lcd* lcd = &g_udc_lcd;
	udc32_t lcd_sec_offset = 0;
	udc_t *value;
    udc_section section;
	lcd->udc = udc_get_udc();

	lcd->item = g_udc_lcd_item;
	
	lcd->panel_device = panel_device;
    lcd_sec_offset = udc_get_lcd_offset();
    value = lcd->udc->buffer + lcd_sec_offset;
	section.buffer = value;
    section.id = *value++;
	section.size = *value;
    lcd->current_section = section;
	lcd->current_lcd_offset = lcd_sec_offset;

	
	UDC_LCD_TRACE("[%s] section.id=0x%x,section.size=0x%x,lcd_sec_offset = %d \n",
		__func__, lcd->current_section.id, lcd->current_section.size, lcd->current_lcd_offset );
	   
	
	udc_lcd_config_panel(lcd, section_id);
	
	return 	lcd;
}





EXPORT_SYMBOL(udc_get_lcd);


