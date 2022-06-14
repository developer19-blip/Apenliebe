// ALPENLIEBE

#include <PID_v1.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <LCD_I2C.h>

#define heatMode 3  // Pin untuk tombol heater mode
#define coolMode 6  // Pin untuk tombol cooler mode
#define buzz 13     // Pin untuk buzzer

int mode; // Inisialisasi variabel

double waktu_selisih; // inisialisasi variabel

uint32_t waktu_awal;  // Inisialisasi variabel
uint32_t waktu_akhir; // Inisialisasi variabel

LCD_I2C lcd(0x27);  // Inisialisasi alamat LCD

// Pemanas
#define ONE_WIRE_H 2  // Pin untuk sensor suhu pemanas
#define SSR_PIN_H 5   // Pin untuk solid state relay pemanas

float suhuSekarang; // Inisialisasi variabel

double Input, Output, Setpoint, init_temp, timeLeft;  // Inisialisasi variabel

uint16_t max_temp = 70; // Suhu tertinggi dalam derajat Celsius
uint8_t soak_time = 10;  // Waktu perendaman dalam menit (ketika telah mencapai waktu pemanasan yang diinginkan)
uint8_t ramp_rate = 2;  // Kecepatan pemanasan dalam derajat Celsius per menit
uint8_t cool_down = 2;  // Kecepatan cooldown dalam derajat Celsius per menit

uint8_t state = 0;      // Inisialisasi untuk kondisi (0. Pemanasan, 1. Idle, 2. Cooldown, 3. Start, 4. Pendinginan)
uint8_t init_read = 1;  // Inisialisasi untuk melewatkan 1 loop

uint32_t PID_interval = 5000; // Interval untuk PID dalam milidetik

PID myPID(&Input, &Output, &Setpoint, 80, 50, 0, DIRECT); // Input, Output, Setpoint, Kp, Ki, Kd, DIRECT

OneWire oneWireH(ONE_WIRE_H);          // Inisialisasi data satu kabel pada pin 2
DallasTemperature sensorSuhuH(&oneWireH); // yang selanjutnya digunakan untuk sensor suhu

// Pendingin
#define ONE_WIRE_C A2 // Pin untuk sensor suhu pemanas
#define SSR_PIN_C 4   // Pin untuk solid state relay pendingin
 
OneWire oneWireC(ONE_WIRE_C);             // Inisialisasi data satu kabel pada pin 2
DallasTemperature sensorSuhuC(&oneWireC); // yang selanjutnya digunakan untuk sensor suhu

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // Inisialisasi untuk menggunakan port serial dengan 9600 bps
  lcd.begin();        // Inisialisasi untuk menggunakan LCD
  lcd.backlight();    // Inisialisasi untuk baklight LCD

  state = 4;  // Inisialisasi awal ke kondisi 4

  pinMode(heatMode, INPUT_PULLUP);  // Pin heater mode sebagai input pull up
  pinMode(coolMode, INPUT_PULLUP);  // Pin cooler mode sebagai input pull up

  pinMode(buzz, OUTPUT);    // Pin buzzer sebagai output
  digitalWrite(buzz, LOW);  // Inisialisasi buzzer (mati)

  // Pemanas
  sensorSuhuH.begin();            // Inisialisasi untuk menggunakan sensor suhu
  pinMode(SSR_PIN_H, OUTPUT);     // Pin solid state relay pemanas sebagai output
  digitalWrite(SSR_PIN_H, HIGH);  // Inisialisasi solid state relay heater (mati)

  // Pendingin
  sensorSuhuC.begin();
  pinMode(SSR_PIN_C, OUTPUT);     // Pin solid state relay pendingin sebagai output
  digitalWrite(SSR_PIN_C, HIGH);  // Inisialisasi solid state relay coller (mati)

  lcd.clear();              // Membersihkan tampilan LCD
  lcd.setCursor(3, 0);      // Menentukan titik awal letak karakter
  lcd.print("WELCOME TO");  // Menampilkan kata
  lcd.setCursor(3, 1);
  lcd.print("ALPENLIEBE");
  delay(4000);              // Tunda 4 detik
}

void loop() {
  // put your main code here, to run repeatedly:
  switch(state){
    case 0:
      run_cycle_time();         // Menjalankan fungsi menghitung waktu
      lcd.clear();              // Membersihkan tampilan LCD
      lcd.setCursor(4, 0);      // Menentukan titik awal letak karakter      
      lcd.print("WARMING");  // Menampilkan kata
      lcd.setCursor(4, 1);      // Menentukan titik awal letak karakter
      lcd.print("FINISHED");    // Menampilkan kata
      delay(3000);
      lcd.clear();
      lcd.setCursor(4 , 0);
      lcd.print("HOLDING");
      lcd.setCursor(5, 1);
      lcd.print("START");
      buzzer();                 // Menjalankan fungsi buzzer
      state = state + 1;        // Melanjutkan ke kondisi berikutnya
      break;
    case 1:
      run_cycle_time();         // Menjalankan fungsi menghitung waktu
      lcd.clear();              // Membersihkan tampilan LCD
      lcd.setCursor(4, 0);      // Menentukan titik awal letak karakter      
      lcd.print("HOLDING");  // Menampilkan kata
      lcd.setCursor(4, 1);      // Menentukan titik awal letak karakter
      lcd.print("FINISHED");    // Menampilkan kata
      delay(3000);
      lcd.clear();
      lcd.setCursor(4 , 0);
      lcd.print("RESTING");
      lcd.setCursor(5, 1);
      lcd.print("START");
      buzzer();                 // Menjalankan funsi buzzer
      state = state + 1;        // Melanjutkan ke kondisi berikutnya
      break;
    case 2:
      run_cycle_time();           // Menjalankan fungsi menghitung waktu
      lcd.clear();                // Membersihkan tampilan LCD
      lcd.setCursor(4, 0);        // Menentukan titik awal letak karakter      
      lcd.print("RESTING");  // Menampilkan kata
      lcd.setCursor(4, 1);        // Menentukan titik awal letak karakter
      lcd.print("FINISHED");      // Menampilkan kata
      buzzer();                   // Menjalankan fungsi buzzer
      state = 4;                  // Kembali ke kondisi awal
      break;
    case 3:
      run_cooler();               // Menjalankan fungsi cooler
      lcd.clear();                // Membersihkan tampilan LCD
      lcd.setCursor(2, 0);        // Menentukan titik awal letak karakter      
      lcd.print("COOLING DOWN");  // Menampilkan kata
      lcd.setCursor(4, 1);        // Menentukan titik awal letak karakter
      lcd.print("FINISHED");      // Menampilkan kata
      buzzer();                   // Menjalankan fungsi buzzer
      state = 4;                  // Kembali ke kondisi awal
      break;
    case 4:
      if(digitalRead(heatMode) == HIGH && digitalRead(coolMode) == HIGH){ // Jika belum memilih mode
        mode = 0;
        lcd.clear();                                                      // Membersihkan tampilan LCD
        lcd.setCursor(5, 0);                                              // Menentukan titik awal letak karakter
        lcd.print("PLEASE");                                              // Menampilkan kata
        lcd.setCursor(2, 1);                                              // Menentukan titik awal letak karakter
        lcd.print("SELECT MODE!");                                        // Menampilkan kata

        if(digitalRead(heatMode) == LOW && digitalRead(coolMode) == HIGH){  // Jika memilih mode heater
          mode = 1;
          lcd.clear();                                                      // Membersihkan tampilan LCD
          lcd.setCursor(2, 0);                                              // Menentukan titik awal letak karakter
          lcd.print("HEATING MODE");                                        // Menampilkan kata
          lcd.setCursor(7, 1);                                              // Menentukan titik awal letak karakter
          lcd.print("ON");                                                  // Menampilkan kata
          delay(3000);                                                      // Tunda 3 detik
    
          while(digitalRead(heatMode) == HIGH){ // Selama nilai dari heatMode sama dengan HIGH
            lcd.clear();                        // Membersihkan tampilan LCD
            lcd.setCursor(5, 0);                // Menentukan titik awal letak karakter      
            lcd.print("PRESS!");                // Menampilkan kata
            lcd.setCursor(2, 1);                // Menentukan titik awal letak karakter
            lcd.print("START HEATER");          // Menampilkan kata
            
            if(digitalRead(heatMode) == LOW){ // Jika nilai dari heatMode LOW
              state = 0;                      // Melanjutkan ke kondisi warming up
            }
          }
        }
  
        else if(digitalRead(heatMode) == HIGH && digitalRead(coolMode) == LOW){ // Jika memilih mode cooler
          mode = 2;
          lcd.clear();                                                          // Membersihkan tampilan LCD
          lcd.setCursor(2, 0);                                                  // Menentukan titik awal letak karakter
          lcd.print("COOLING MODE");                                            // Menampilkan kata
          lcd.setCursor(7, 1);                                                  // Menentukan titik awal letak karakter
          lcd.print("ON");                                                      // Menampilkan kata
          delay(3000);                                                          // Tunda 3 detik
    
          while(digitalRead(coolMode) == HIGH){ // Selama nilai dari button sama dengan HIGH
            lcd.clear();                        // Membersihkan tampilan LCD
            lcd.setCursor(5, 0);                // Menentukan titik awal letak karakter      
            lcd.print("PRESS!");                // Menampilkan kata
            lcd.setCursor(2, 1);                // Menentukan titik awal letak karakter
            lcd.print("START COOLER");          // Menampilkan kata
            
            if(digitalRead(coolMode) == LOW){ // Jika nilai dari coolMode LOW
              state = 3;                      // Melanjutkan ke kondisi cooling
            }
          }
        }
      }
      break;
  }
}

// Fungsi menyalakan buzzer
void buzzer(){
  for(int i = 0; i < 3; i++){ // Selama nilai dari i kurang dari 3
    digitalWrite(buzz, 1);    // Menyalakan buzzer
    delay(1000);              // Tunda 1 detik
    digitalWrite(buzz, 0);    // Mematikan buzzer
    delay(500);               // Tunda 0.5 detik
  }
}

// Fungsi menampilkan pada LCD
void Display(){
  String temp;  // Inisialisasi variabel
  String tleft; // Inisialisasi variabel
  
  if(mode == 1){                // Jika memilih mode heater
    temp = (String)ambilSuhuH(); // Menyimpan nilai suhu (derajat celsius) dalam variabel temp
    tleft = (String)timeLeft;   // Menyimpan nilai waktu (menit) dalam variabel tleft
    Serial.print(ambilSuhuH());
    Serial.print(",");
    Serial.println(state);
  }
  
  else if(mode == 2){             // Jika memilih mode cooler
    temp = (String)ambilSuhuC();  // Menyimpan nilai suhu (derajat celsius) dalam variabel temp
    tleft = waktu_selisih;        // Menyimpan nilai waktu (menit) dalam variabel tleft
    Serial.println(ambilSuhuC());
    
  }
  
  lcd.clear();                          // Membersihkan tampilan OLED  
  lcd.setCursor(0, 0);                  // Menentukan titik awal letak karakter
  lcd.print("Temp   : " + temp + " C");  // Menampilkan kata
  lcd.setCursor(0, 1);                  // Menentukan titik awal letak karakter
  lcd.print("Time   : " + tleft + " m"); // Menampilkan kata
}

// Pemanas
// Fungsi membaca suhu heater
float ambilSuhuH(){
  sensorSuhuH.requestTemperatures();            // Memanggil class dari library
  float suhu = sensorSuhuH.getTempCByIndex(0);  // Menyimpan nilai suhu (derajat celsius) dalam variabel suhu
  return suhu;                                  // Melanjutkan nilai suhu (float type) jika fungsi dipanggil
}

// Fungsi menghitung waktu
void run_cycle_time(void){
  double cycle_time;      // Inisialisasi variabel
  waktu_awal = millis();  // Menyimpan waktu dalam milidetik dalam variabel waktu_awal
 
  if(state == 0){                                   // Jika kondisi menaikan suhu
    ambilSuhuH();                                   // Menjalankan fungsi membaca suhu
    init_temp = ambilSuhuH();                       // Menyimpan nilai suhu (derajat celsius) dalam variabel init_temp
    cycle_time = (max_temp - init_temp)/ramp_rate;  // Menghitung waktu yang diperlukan untuk menaikan suhu dan menyimpannya (menit) dalam variabel cycle_time
  }
  else if(state == 1){      // Jika kondisi menjaga suhu
    ambilSuhuH();
    cycle_time = soak_time; // Menyimpan nilai soak_time (menit) dalam variabel cycle_time
  }
  else if(state == 2){                                  // Jika kondisi menurunkan suhu
    ambilSuhuH();
    cycle_time = (ambilSuhuH() - init_temp)/cool_down;  // Menghitung waktu yang diperlukan untuk menurunkan suhu dan menyimpannya (menit) dalam variabel cycle_time
  }

  
  waktu_akhir = millis();                                   // Menyimpan waktu dalam milidetik dalam variabel waktu_akhir
  waktu_selisih = float(waktu_akhir - waktu_awal) / 60000;  // Menghitung waktu yang telah berjalan dan menyimpannya (menit) dalam variabel waktu_selisih
 
  while(waktu_selisih < cycle_time){                      // Selama waktu_selisih kurang dari cycle_time
    if(state == 0){                                       // Jika kondisi menaikan suhu
      Setpoint = (waktu_selisih * ramp_rate) + init_temp; // Menghitung nilai setpoint PID dan menyimpannya dalam variabel Setpoint
      run_PID(80, 50, 0, 500, PID_interval);              // Menjalankan fungsi PID
    }
    else if(state == 1){                      // Selain itu, jika kondisi menjaga suhu
      Setpoint = max_temp;                    // Menyimpan nilai max_temp dalam variabel Setpoint
      run_PID(80, 50, 0, 500, PID_interval);  // Menjalankan fungsi PID
    }
    else if(state == 2){                                  // Selain itu, jika kondisi menurunkan suhu
      Setpoint = max_temp - (waktu_selisih * cool_down);  // Menghitung nilai setpoint PID dan menyimpannya dalam variabel Setpoint
      digitalWrite(SSR_PIN_H, HIGH);                      // Relay mati
    }

    timeLeft = cycle_time - waktu_selisih;  // Menghitung sisa waktu dan menyimpannya (menit) dalam variabel timeLeft
    
    waktu_akhir = millis();                                   // Menyimpan waktu dalam milidetik dalam variabel waktu_akhir
    waktu_selisih = float(waktu_akhir - waktu_awal) / 60000;  // Menghitung waktu yang telah berjalan dan menyimpannya (menit) dalam variabel waktu_selisih
    
    Display();  // Menampilkan informasi pada LCD
  }
}

// Fungsi menjalankan PID
void run_PID(double Kp, double Ki, double Kd, uint16_t WindowSize, uint32_t time_interval){
  double ratio;             // Inisialisasi variabel
  uint32_t windowStartTime; // Inisialisasi variabel

  myPID.SetOutputLimits(0, WindowSize); // Menentukan limit output PID
  myPID.SetTunings(Kp, Ki, Kd);         // Menentukan tuning PID
  myPID.SetMode(AUTOMATIC);             // Menentukan mode PID

  if(init_read){
    init_read = 0;
    Setpoint = Setpoint + 1;
  }
  else{
    ambilSuhuH();
  }

  windowStartTime = millis(); // Menyimpan waktu dalam milidetik dalam variabel windowStartTime
  
  Input = ambilSuhuH(); // Menyimpan nilai suhu (derajat celsius) dalam variabel Input
  myPID.Compute();      // Menghitung PID
  
  ratio = Output / WindowSize;  // Menghitung nilai ratio dan menyimpannya dalam variabel ratio

  if(millis() - windowStartTime < time_interval * ratio){ // Jika nilai dari hasil millis() dikurang windowStartTime kurang dari time_interval dikali ratio
    digitalWrite(SSR_PIN_H, LOW);                         // Relay hidup
  }

  else if(millis() - windowStartTime < time_interval){  // Selain itu, ika nilai dari hasil millis() dikurang windowStartTime kurang dari time_interval
    digitalWrite(SSR_PIN_H, HIGH);                      // Relay mati
  }
}

// Pendingin
// Fungsi membaca suhu cooler
float ambilSuhuC(){
  sensorSuhuC.requestTemperatures();            // Memanggil class dari library
  float suhu = sensorSuhuC.getTempCByIndex(0);  // Menyimpan nilai suhu (derajat celsius) dalam variabel suhu
  return suhu;                                  // Melanjutkan nilai suhu (float type) jika fungsi dipanggil
}

// Fungsi cooler
void run_cooler(){
  waktu_awal = millis();
  suhuSekarang = ambilSuhuC();  // Menyimpan nilai suhu (derajat celsius) dalam variabel suhuSekarang
  
  while(digitalRead(coolMode) == HIGH){ // Selama nilai coolMode sama dengan HIGH
    
    digitalWrite(SSR_PIN_C, LOW); // Relay hidup

    waktu_akhir = millis();
    waktu_selisih = float(waktu_akhir - waktu_awal) / 60000;

    Display();  // Menampilkan informasi pada LCD

    if(digitalRead(coolMode) == LOW){
      digitalWrite(SSR_PIN_C, HIGH); // Relay mati
    }
  }
}
