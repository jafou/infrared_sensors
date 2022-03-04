/*
 * Attention! I commented out the alpha_ij array, so if you're going to compile the sketch you'll get for sure an error.
 * You should replace all 64 values with the alpha_ij calculated using the values stored in your MLX90620's EEPROM. 
 * I suggest you to make an EEPROM dump, print it on the Serial port and store it in a file. From there, with the help of a spreadsheet (Libreoffice, Google Docs, Excel...) calculate your own alpha_ij values. 
 * Please also pay attention to your emissivity value: since in my case it was equal to 1, to save SRAM i cut out that piece of calculation. You need to restore those lines if your emissivity value is not equal to 1. 
 */

#include <i2cmaster.h>
#include <CAN.h>
#include <SPI.h>
#include <SoftwareSerial.h>

#define BUS_SPEED 500 // CAN speed

// I2C and MLX90621 variables
int freq = 8;  //Set this value to your desired refresh frequency

int IRDATA[64]; // The individual pixel IR data (RAM read)
byte CFG_LSB, CFG_MSB, PTAT_LSB, PTAT_MSB, CPIX_LSB, CPIX_MSB, PIX_LSB, PIX_MSB;
int PIX, CPIX;
float ta, to, emissivity, k_t1, k_t2, v_th, tgc;
float temperatures[64];
int count=0;
unsigned int PTAT;
float A_cp, B_cp;
int A_common;
unsigned int B_i_scale, delta_A_i_scale, alpha_0_scale, delta_alpha_scale;
unsigned int k_t1_scale, k_t2_scale;
unsigned int delta_A_ij[64];
unsigned int bit_resolution; // The bit resolution stored in configuration register [5:4]
float A_ij[64],B_ij[64];
float alpha_cp = 4.3E-9;
float alpha_ij[64] = {6.84577E-08,7.38128E-08,7.47441E-08,6.68279E-08,7.56754E-08,8.24275E-08,8.07977E-08,7.33471E-08,8.21947E-08,8.89468E-08,8.87139E-08,8.10305E-08,8.75498E-08,9.45347E-08,9.36034E-08,8.38245E-08,9.22064E-08,9.73287E-08,9.59317E-08,8.68513E-08,9.33705E-08,9.94241E-08,9.9657E-08,9.03437E-08,9.56989E-08,1.03149E-07,1.03149E-07,9.03437E-08,9.61645E-08,1.03848E-07,1.03382E-07,9.45347E-08,9.47675E-08,1.01985E-07,1.04314E-07,9.33705E-08,9.56989E-08,1.03382E-07,1.02917E-07,9.17407E-08,9.45347E-08,1.02451E-07,1.00821E-07,9.08094E-08,9.19736E-08,9.87256E-08,9.91913E-08,8.75498E-08,8.77826E-08,9.59317E-08,9.52332E-08,8.40573E-08,8.12634E-08,9.08094E-08,8.77826E-08,7.77709E-08,7.42784E-08,8.3126E-08,8.00992E-08,7.17173E-08,6.68279E-08,7.38128E-08,7.31143E-08,6.38011E-08};  //<-- REPLACE THIS VALUES WITH YOUR OWN!
float v_ir_tgc_comp; // Contains the V_ir_TGC_compensated values of the 64 pixels.
float v_ir_comp; // Contains the V_ir_compensated values of the 64 pixels.
float v_ir_offset_comp; // Contains the V_ir_offset_compensated values of the 64 pixels.
float v_ir_cp_offset_comp;
float v_ir_normalized;
float overall_temperature;

// CAN Variables
byte length, tx_status;
unsigned short frame_id;
byte frame_data[8];  //data frame of the first CAN message (4 2-byte variables)
unsigned int value_to_send;
byte data1[2];

// checked that all the data is read correctly from the EEPROM
void read_EEPROM_MLX90621(){
  byte EEPROM_DATA[256]; // The EEPROM is organized as a 2kbit, 256x8 memory
  i2c_start_wait(0xA0); 
  i2c_write(0x00);
  i2c_rep_start(0xA1);
  for(int i=0;i<=255;i++){
    EEPROM_DATA[i] = i2c_readAck();
  }
  i2c_readNak();
  i2c_stop();
  varInitialization(EEPROM_DATA); // stores all the EEPROM data into program variables.
  // Print out all the EEPROM DATA for debugging puproses.
  for (int i=0;i<64;i++){
    //Serial.print(i);
    //Serial.print(" = ");
    Serial.println(EEPROM_DATA[0x80 + i]);
  }
  write_trimming_value(EEPROM_DATA[247]);
}

// checked
void write_trimming_value(byte val){
  i2c_start_wait(0xC0);
  i2c_write(0x04); 
  i2c_write((byte)val-0xAA); 
  i2c_write(val);   
  i2c_write(0x56);  
  i2c_write(0x00);  
  i2c_stop();
}


void config_MLX90621_Hz(int Hz){
  byte Hz_LSB;
  switch(Hz){
    case 0:
      Hz_LSB = 0b00111111;
      break;
    case 1:
      Hz_LSB = 0b00111110;
      break;
    case 2:
      Hz_LSB = 0b00111101;
      break;
    case 4:
      Hz_LSB = 0b00111100;
      break;
    case 8:
      Hz_LSB = 0b00111011;
      break;
    case 16:
      Hz_LSB = 0b00111010;
      break;
    case 32:
      Hz_LSB = 0b00111001;
      break;
    case 64:
      Hz_LSB = 0b00111000;
      break;
    case 128:
      Hz_LSB = 0b00110111;
      break;
    case 256:
      Hz_LSB = 0b00110110;
      break;
    case 512:
      Hz_LSB = 0b00110101;
      break;
    default:
      Hz_LSB = 0b00111110;
  }
  i2c_start_wait(0xC0);
  i2c_write(0x03);    
  i2c_write((byte)Hz_LSB-0x55); 
  i2c_write(Hz_LSB);   
  i2c_write(0xF9); // The value of the high byte of the configuration register is set to 0x46 for the MLX90621. Remains to be checked
  i2c_write(0x4E);  
  i2c_stop();
}

void read_PTAT_data_MLX90621(){
  i2c_start_wait(0xC0);
  i2c_write(0x02);
  i2c_write(0x40); // changed according to datasheet
  i2c_write(0x00);
  i2c_write(0x01);
  i2c_rep_start(0xC1);
  PTAT_LSB = i2c_readAck();
  PTAT_MSB = i2c_readAck();
  i2c_readNak();
  i2c_stop();
  PTAT = ((unsigned int)PTAT_MSB << 8) + PTAT_LSB; // type casting not necessary
}

// seems correct
void calculate_TA(){ 
  ta = (-k_t1 + sqrt(sq(k_t1) - (4 * k_t2 * (v_th - (float)PTAT))))/(2*k_t2) + 25; //it's much more simple now, isn't it? :)
}

// seems ok
void calculate_TO(){
  v_ir_cp_offset_comp = (float)CPIX - (A_cp + B_cp *(ta-25)); // checked
  for (int i=0; i<64; i++){
    v_ir_offset_comp = IRDATA[i] - (A_ij[i] + B_ij[i] *(ta-25));// checked
    v_ir_tgc_comp = v_ir_offset_comp - tgc*v_ir_cp_offset_comp; // checked
    v_ir_normalized = v_ir_tgc_comp / (alpha_ij[i] - (tgc * alpha_cp)); // checked
    v_ir_comp= v_ir_normalized / emissivity; // can easily check emissivity of brake discs by changing this value 
    
    temperatures[i] = sqrt(sqrt(v_ir_comp + pow((ta + 273.15),4))) - 273.15; //changed from the original
  }
}

// working
void read_IR_ALL_MLX90621(){
  i2c_start_wait(0xC0);
  i2c_write(0x02);      
  i2c_write(0x00);     
  i2c_write(0x01);       
  i2c_write(0x40);       
  i2c_rep_start(0xC1);
  for(int i=0;i<=63;i++){
    PIX_LSB = i2c_readAck(); 
    PIX_MSB = i2c_readAck(); 
    IRDATA[i] = (PIX_MSB << 8) + PIX_LSB;
  }
  i2c_readNak();
  i2c_stop();
}

//working
void read_CPIX_Reg_MLX90621(){
  i2c_start_wait(0xC0);
  i2c_write(0x02);
  i2c_write(0x41);
  i2c_write(0x00);
  i2c_write(0x01);
  i2c_rep_start(0xC1);
  CPIX_LSB = i2c_readAck();
  CPIX_MSB = i2c_readAck();
  i2c_readNak();
  i2c_stop();
  CPIX = (CPIX_MSB << 8) + CPIX_LSB;
}

// working
void read_Config_Reg_MLX90621(){
  i2c_start_wait(0xC0);
  i2c_write(0x02);
  i2c_write(0x92);
  i2c_write(0x00);
  i2c_write(0x01);
  i2c_rep_start(0xC1);
  CFG_LSB = i2c_readAck();
  CFG_MSB = i2c_readAck();
  i2c_readNak();
  i2c_stop();
}

// working
void check_Config_Reg_MLX90621(){
  read_Config_Reg_MLX90621();
  // The following if statement checks if the MD has written "1" to the POR/Brown out flag 
  if (!((CFG_MSB & 0b00000100) == 0x04)){
    config_MLX90621_Hz(freq);
  }
}


void varInitialization(byte EEPROM_DATA[]){
  // For 2-byte signed integers, the following equation works. No need for "if" statement as
  // for the 1-byte signed values;
  k_t1_scale = (EEPROM_DATA[210] & 0b11110000) >> 4;
  k_t2_scale = EEPROM_DATA[210] & 0b00001111;
  // Read 2 bits from the configuration register that correspond to the bit resolution
  // The address is chosen at 0xF5 (low byte)
  read_Config_Reg_MLX90621();
  bit_resolution = (CFG_LSB & 0b00110000) >> 4;
  // The following equations have been changed in order to take into account the bit resolution.
  v_th = ((EEPROM_DATA[219] <<8) + EEPROM_DATA[218]) / pow(2.0, 3-bit_resolution);
  k_t1 = ((EEPROM_DATA[221] <<8) + EEPROM_DATA[220])/ (pow(2.0,k_t1_scale)*pow(2.0,3-bit_resolution)); // refer to page 14 of the datasheet
  k_t2 =((EEPROM_DATA[223] <<8) + EEPROM_DATA[222])/ (pow(2.0,k_t2_scale + 10)*pow(2.0,3-bit_resolution)); 


  A_common = (EEPROM_DATA[209] <<8) + EEPROM_DATA[208];
  // Alternative for A_cp
  A_cp = (float)(EEPROM_DATA[212] << 8) + EEPROM_DATA[211];
  A_cp = A_cp/pow(2.0, 3-bit_resolution);
  B_cp = (float)EEPROM_DATA[213]; // This is ok, since B_cp is one byte
  if(B_cp > 127){
    B_cp = B_cp - 256;
  }
  tgc = EEPROM_DATA[216]; // same here, everything ok
  if(tgc > 127){
    tgc = tgc - 256;
  }
  tgc = tgc/32.0;
  // In order for this division to work, we have to add a ".0" to the end of the constant
  // so that the result is floating point.
  emissivity = ((unsigned int)(EEPROM_DATA[229] << 8) + EEPROM_DATA[228])/32768.0;

  for(int i=0;i<=63;i++){
    delta_A_ij[i] = EEPROM_DATA[i];

    // Working for signed values!!! Type casting doesn't work ((int) EEPROM_DATA[])
    B_ij[i] = EEPROM_DATA[64+i];
    if(B_ij[i] > 127){
      B_ij[i] = B_ij[i] - 256;
    }
  }

  alpha_0_scale = EEPROM_DATA[226];
  delta_A_i_scale = (EEPROM_DATA[217] & 0b11110000) >> 4;
  B_i_scale = EEPROM_DATA[217] & 0b00001111;
  delta_alpha_scale = EEPROM_DATA[227];

  for (int i=0;i<=63;i++){
    A_ij[i] = ((float)A_common + (delta_A_ij[i] * pow(2.0, delta_A_i_scale)))/pow(2.0,3-bit_resolution);
    B_ij[i] = B_ij[i]/(pow(2.0,B_i_scale)*pow(2.0,3-bit_resolution));
  }
  B_cp = B_cp/(pow(2.0,B_i_scale)*pow(2.0,3-bit_resolution));
  
}

void calculate_average_temperature(){
  float sum =0;
  for(int i=0;i<=63;i++){
    sum += temperatures[i];
  }
  overall_temperature = sum/64.0;
  //Serial.write((unsigned int)overall_temperature * 10);
  Serial.println(overall_temperature);
}

void int2Bytes(unsigned int val,byte* bytes_array){
  // Create union of shared memory space
  union {
    unsigned int int_variable;
    byte temp_array[2];
  } u;
  // Overwrite bytes of union with float variable
  u.int_variable = val;
  // Assign bytes to input array
  memcpy(bytes_array, u.temp_array, 2);
}

void setup(){
  Serial.begin(9600);
  CAN.begin();                //initialization of the CAN class which
  CAN.baudConfig(BUS_SPEED);  //initializes the SPI communication of the microcontroller
  CAN.setMode(NORMAL);        //with the CAN controller (MCP2515)
  i2c_init(); 
  PORTC = (1 << PORTC4) | (1 << PORTC5);//Enable pull up resistors
  delay(5);
  config_MLX90621_Hz(freq);
  read_EEPROM_MLX90621();
}

void loop(){
  if(count ==0){ //TA refresh is slower than the pixel readings, I'll read the values and computate them not every loop. 
    read_PTAT_data_MLX90621();// working!!
    calculate_TA(); // working
    check_Config_Reg_MLX90621();//working
    //Serial.println(ta);
    //delay(200);
  }
  count++;
  if(count >=8){
    count = 0;
  }
  read_IR_ALL_MLX90621();
  /*for (int i=0;i<64;i++){
    Serial.println(IRDATA[i]);
    delay(200);
  }*/
  read_CPIX_Reg_MLX90621();
  calculate_TO();
  calculate_average_temperature();

  // Send the acquired data over to the CAN Bus in order to be read by the Telemetry board.
  
  //Convert each value we want to send, to 2 independent bytes.
  //After the call of this function, data#[0] is the low byte and data#[1] is the high byte.
  int2Bytes(value_to_send,&data1[0]);

  frame_id = 0x5F9; //
  length = 8;

  // The frame data is sent low byte first. For each one of the 4 variables,
  // the respective slot of the frame data is filled accordingly.
  frame_data[0] = data1[0];
  frame_data[1] = data1[1];
  
  CAN.load_ff_0(length, frame_id, frame_data);
  CAN.send_0();
  delay(50); // Reduce the delay for testing
  
}
