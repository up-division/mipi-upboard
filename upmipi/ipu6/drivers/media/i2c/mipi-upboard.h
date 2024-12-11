#include <linux/dmi.h>
#include <linux/gpio.h>

#define UPBOARD_CAM1_RESET "CAM1_RST"
#define UPBOARD_CAM2_RESET "CAM2_RST"
#define UPBOARD_CRD1_POWER_ENABLE "CRD1_PWREN"
#define UPBOARD_CRD2_POWER_ENABLE "CRD2_PWREN"

struct ctrl_gpio {
	const char *name;
	int offset;
};

struct upmipi_gpios {
	struct ctrl_gpio *gpios;
	int ngpio;
};

/*  UP Board MIPI CSI2 Control GPIO  */
const struct ctrl_gpio upxadl_ctrl_gpios[] = {
        {
                .name = UPBOARD_CAM1_RESET,
                .offset = 357,  //R5
        },
        {
                .name = UPBOARD_CRD1_POWER_ENABLE,
                .offset = 23,  //B23       
        },
        {
                .name = UPBOARD_CAM2_RESET,
                .offset = 335,  //E15        
        },
        {
                .name = UPBOARD_CRD2_POWER_ENABLE,
                .offset = 336, //E16  
        },
};

const struct ctrl_gpio upxmtl_ctrl_gpios[] = {
        {
                .name = UPBOARD_CAM1_RESET,
                .offset = 397,  //D13
        },
        {
                .name = UPBOARD_CRD1_POWER_ENABLE,
                .offset = 72,  //C8       
        },
        {
                .name = UPBOARD_CAM2_RESET,
                .offset = 113,  //A17        
        },
        {
                .name = UPBOARD_CRD2_POWER_ENABLE,
                .offset = 115, //A19  
        },
};

const struct upmipi_gpios upxadl_gpios = {
        .gpios = upxadl_ctrl_gpios,
        .ngpio = ARRAY_SIZE(upxadl_ctrl_gpios),
};

const struct upmipi_gpios upxmtl_gpios = {
        .gpios = upxmtl_ctrl_gpios,
        .ngpio = ARRAY_SIZE(upxmtl_ctrl_gpios),
};

static const struct dmi_system_id upmipi_dmi_table[] = {
	{
		.matches = { /* UP Xtreme i12 */
			DMI_EXACT_MATCH(DMI_SYS_VENDOR, "AAEON"),
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "UPX-ADLP01"),
		},
		.driver_data = (void*)&upxadl_gpios,
	},		
	{
		.matches = {
			DMI_EXACT_MATCH(DMI_SYS_VENDOR, "AAEON"),
			DMI_EXACT_MATCH(DMI_BOARD_NAME, "UPX-MTL01"),
		},
		.driver_data = (void*)&upxmtl_gpios,
	},
	{},
};

inline struct upmipi_gpios* mipi_upboard_gpios(void)
{
	/* check board id to arrange driver data for gpio ctrl*/
	const struct dmi_system_id *upmipi_id = dmi_first_match(upmipi_dmi_table);
	if(upmipi_id)
	{
	  return (struct upmipi_gpios*)upmipi_id->driver_data;
	}
        return NULL;
}


