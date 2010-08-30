/******************************************************************************/
/*                                                    */
/*  Spark Fun Electronics                                           */
/******************************************************************************/

#define GPS_CHECKSUM(mstr, mx)  {mx=0; for(int mi = 0; mi < strlen(mstr);mi++ ) mx ^= mstr[mi];}
/*
*    GPS FUNCTIONS
*/
void GPS_init_strings(void);

void config_gps_msgs(unsigned char const type, unsigned char const freq);
void disable_all_gps_msgs(void);
void enable_gps_rmc_msgs(unsigned char freq);
void disable_gps_rmc_msgs(void);
void enable_gps_gga_msgs(unsigned char freq);
void disable_gps_gga_msgs(void);
void configure_gps_waas(unsigned char const enable);
void enable_waas(void);
void disable_waas(void);
