/*****************************************************************
    M5Stack Gray と GPS モジュールを使って
    GPS座標・加速度・ジャイロセンサーの出力をLCDに表示する
    公式のサンプルコード「FullExample」がベース
    
    TinyGPSPlus:http://arduiniana.org/libraries/tinygpsplus/ からダウンロード
      スケッチ→ライブラリをインクルード→ZIP形式のライブラリをインストール
*****************************************************************/

#define M5STACK_MPU9250 

#include <M5Stack.h>
#include <TinyGPS++.h>

static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
HardwareSerial ss(2);
float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;

float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;

float pitch = 0.0F;
float roll  = 0.0F;
float yaw   = 0.0F;

float temp = 0.0F;

void setup()
{
  M5.begin();
  dacWrite(25, 0); /* スピーカーオフにしてノイズを消す */
  M5.Power.begin();
  ss.begin(GPSBaud);
  M5.IMU.Init();
  M5.Lcd.setTextSize(1);
}

void loop()
{
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextColor(WHITE, BLACK);

  M5.Lcd.print("Num of Sat[-]:");
  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  M5.Lcd.print("HDOP[-]:");
  printInt(gps.hdop.value(), gps.hdop.isValid(), 5);
  M5.Lcd.print("DateTime[-]:");
  printDateTime(gps.date, gps.time);
  M5.Lcd.println();

  M5.Lcd.setTextColor(BLUE, BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.print("LAT[-]:");
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  M5.Lcd.print("LNG[-]:");
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
//  printInt(gps.location.age(), gps.location.isValid(), 5);
  M5.Lcd.print("ALT[-]:");
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
//  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
//  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
//  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ", 6);
//  printInt(gps.charsProcessed(), true, 6);
//  printInt(gps.sentencesWithFix(), true, 10);
//  printInt(gps.failedChecksum(), true, 9);

  M5.Lcd.println();

  if (millis() > 5000 && gps.charsProcessed() < 10)
    M5.Lcd.println(F("No GPS data received: check wiring"));

  int div_num = 10; //平均化回数
  int total_delay_num = 1000 / div_num; //平均化して1secになるようにループ回数を設定

  for(int ii = 0;ii < total_delay_num;ii++){
    float ave_accX = 0.0F;
    float ave_accY = 0.0F;
    float ave_accZ = 0.0F;
    float ave_gyroX = 0.0F;
    float ave_gyroY = 0.0F;
    float ave_gyroZ = 0.0F;
    float ave_pitch = 0.0F;
    float ave_roll  = 0.0F;
    float ave_yaw   = 0.0F;
    float ave_temp = 0.0F;

    //ジャイロ、加速度、温度センサーの平均化
    for(int jj = 0;jj < div_num;jj++){
      M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
      M5.IMU.getAccelData(&accX,&accY,&accZ);
      M5.IMU.getAhrsData(&pitch,&roll,&yaw);
      M5.IMU.getTempData(&temp);

      ave_accX  += accX;
      ave_accY  += accY;
      ave_accZ  += accZ;
      ave_gyroX += gyroX;
      ave_gyroY += gyroY;
      ave_gyroZ += gyroZ;
      ave_pitch += pitch;
      ave_roll  += roll;
      ave_yaw   += yaw;
      ave_temp  += temp;

      delay(1);
    }

    ave_accX  /= div_num;
    ave_accY  /= div_num;
    ave_accZ  /= div_num;
    ave_gyroX /= div_num;
    ave_gyroY /= div_num;
    ave_gyroZ /= div_num;
    ave_pitch /= div_num;
    ave_roll  /= div_num;
    ave_yaw   /= div_num;
    ave_temp  /= div_num;
  
    M5.Lcd.setCursor(0, 90);
    M5.Lcd.setTextColor(GREEN , BLACK);
    M5.Lcd.println("GYRO XYZ[o/s]");
    M5.Lcd.printf("%6.2f %6.2f %6.2f          ", ave_gyroX, ave_gyroY, ave_gyroZ);
    M5.Lcd.println("");
    M5.Lcd.println("ACC[G]");
    M5.Lcd.printf("%5.2f %5.2f %5.2f          ", ave_accX, ave_accY, ave_accZ);
    M5.Lcd.println("");
    M5.Lcd.println("P-R-Y[deg]");
    M5.Lcd.printf("%5.2f %5.2f %5.2f          ", ave_pitch, ave_roll, ave_yaw);
    M5.Lcd.println("");
//    M5.Lcd.printf("Temperature[C] %.2f", ave_temp);
//    M5.Lcd.println("");
  }
}

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do{
    while (ss.available())
      gps.encode(ss.read());
  }while(millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if(!valid){
    while (len-- > 1)
      M5.Lcd.print('*');
    M5.Lcd.print(' ');
  }else{
    M5.Lcd.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i){
      M5.Lcd.print(' ');
    }
  }
  M5.Lcd.println();
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if(valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for(int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if(len > 0) 
    sz[len-1] = ' ';
  M5.Lcd.print(sz);
  M5.Lcd.println();
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if(!d.isValid()){
    M5.Lcd.print(F("********** "));
  }else{
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    M5.Lcd.print(sz);
  }
  
  if(!t.isValid()){
    M5.Lcd.print(F("******** "));
  }else{
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    M5.Lcd.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for(int i=0; i<len; ++i){
    M5.Lcd.print(i<slen ? str[i] : ' ');
  }
  M5.Lcd.println();
  smartDelay(0);
}
