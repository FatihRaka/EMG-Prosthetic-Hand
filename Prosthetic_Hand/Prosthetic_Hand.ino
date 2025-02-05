#include "modelcnn3.h" // Ganti dengan model Anda
#include <tflm_esp32.h>
#include <eloquent_tinyml.h>
#include <ESP32Servo.h>
#include <Fuzzy.h>

#define ARENA_SIZE 10000 // memory arena size for the model
#define RXD2 16
#define TXD2 17

Eloquent::TF::Sequential<TF_NUM_OPS, ARENA_SIZE> tf;

float min_wl = 1021.500000;
float max_wl = 2190.000000;
float min_rms = 179.685609;
float max_rms = 225.541068;
float min_mav = 163.858333;
float max_mav = 219.137500;
float min_amp = 125.375000;
float max_amp = 242.500000;

int errorValue = 0;  // Variabel untuk error
int changeInErrorValue = 0; // Variabel untuk change in error
int pos = 1;  // Posisi servo

// Fuzzy
Fuzzy *fuzzy = new Fuzzy();


// // FuzzySet untuk output (-16 sampai 16)
FuzzySet *outNB = new FuzzySet(-16, -16, -12, -8);
FuzzySet *outNM = new FuzzySet(-12, -8, -8, -4);
FuzzySet *outNS = new FuzzySet(-8, -4, -4, 0);
FuzzySet *outZE = new FuzzySet(-4, 0, 0, 4);
FuzzySet *outPS = new FuzzySet(0, 4, 4, 8);
FuzzySet *outPM = new FuzzySet(5, 8, 8, 12);
FuzzySet *outPB = new FuzzySet(8, 12, 16, 16);

// FuzzySet untuk error (-30 sampai 30)
FuzzySet *eNB = new FuzzySet(-30, -30, -15, -10);
FuzzySet *eNM = new FuzzySet(-15, -10, -10, -5);
FuzzySet *eNS = new FuzzySet(-10, -5, -5, 0);
FuzzySet *eZE = new FuzzySet(-5, 0, 0, 5);
FuzzySet *ePS = new FuzzySet(0, 5, 5, 10);
FuzzySet *ePM = new FuzzySet(5, 10, 10, 15);
FuzzySet *ePB = new FuzzySet(10, 15, 30, 30);

// // FuzzySet untuk change in error (-30 sampai 30)
FuzzySet *cNB = new FuzzySet(-30, -30, -15, -10);
FuzzySet *cNM = new FuzzySet(-15, -10, -10, -5);
FuzzySet *cNS = new FuzzySet(-10, -5, -5, 0);
FuzzySet *cZE = new FuzzySet(-5, 0, 0, 5);
FuzzySet *cPS = new FuzzySet(0, 5, 5, 10);
FuzzySet *cPM = new FuzzySet(5, 10, 10, 15);
FuzzySet *cPB = new FuzzySet(10, 15, 30, 30);


// FuzzyInput untuk error dan change in error
FuzzyInput *error = new FuzzyInput(1);
FuzzyInput *changeInError = new FuzzyInput(2);

// FuzzyOutput untuk output kontrol
FuzzyOutput *output = new FuzzyOutput(1);

// Servo
Servo servoJempol, servoTelunjuk, servoTengah, servoManis, servoKelingking;
int servoPins[5] = {9, 10, 11, 12, 13}; // Pin untuk setiap servo

// Buffer untuk menyimpan data sementara
const int numChannels = 8;
const int maxSets = 30;
int channelData[numChannels][maxSets];
int dataCount[numChannels] = {0};

// Variabel fitur untuk per channel
float mavValues[numChannels];
float rmsValues[numChannels];
float wlValues[numChannels];
float amplitudeFirstBurst[numChannels];

Servo* getServo(int index) {
  switch (index) {
    case 0: return &servoJempol;
    case 1: return &servoTelunjuk;
    case 2: return &servoTengah;
    case 3: return &servoManis;
    case 4: return &servoKelingking;
    default: return &servoJempol;  // Default ke servo1 jika terjadi kesalahan
  }
}

// Fungsi normalisasi ke skala 0-255
float normalizeTo255(float value, float min, float max) {
    return (float)((((value - min) / (max - min)) * 255.0));
}

// Fungsi untuk menghitung Mean Absolute Value (MAV)
float calculateMAV(int data[], int length) {
    float sumOfAbsoluteValues = 0.0;
    for (int i = 0; i < length; i++) {
        sumOfAbsoluteValues += abs(data[i]);
    }
    return sumOfAbsoluteValues / length;
}

// Fungsi untuk menghitung Root Mean Square (RMS)
float calculateRMS(int data[], int length) {
    float sumOfSquares = 0.0;
    for (int i = 0; i < length; i++) {
        sumOfSquares += data[i] * data[i];
    }
    return sqrt(sumOfSquares / length);
}

// Fungsi untuk menghitung Waveform Length (WL)
float calculateWaveformLength(int data[], int length) {
    float waveformLength = 0.0;
    for (int i = 1; i < length; i++) {
        waveformLength += abs(data[i] - data[i - 1]);
    }
    return waveformLength;
}

// Fungsi untuk menghitung Amplitudo Burst Pertama
float calculateAmplitudeFirstBurst(int data[], int length, float threshold) {
    for (int i = 0; i < length; i++) {
        if (abs(data[i]) > threshold) {
            return abs(data[i]);
        }
    }
    return 0.0; // Jika tidak ada burst yang melebihi threshold
}

void moveServoWithFuzzy(Servo* servo, int finalPosition) {
    int currentFingerPosition;
    int errorValue = 0;
    unsigned long fingerStartTime = millis();
    
    // Identifikasi servo berdasarkan alamat memori
    const char* servoName;
    if (servo == &servoJempol) {
        servoName = "Servo Ibu Jari";
    } else if (servo == &servoTelunjuk) {
        servoName = "Servo Telunjuk";
    } else if (servo == &servoTengah) {
        servoName = "Servo Jari Tengah";
    } else if (servo == &servoManis) {
        servoName = "Servo Jari Manis";
    } else if (servo == &servoKelingking) {
        servoName = "Servo Kelingking";
    } else {
        servoName = "Servo Tidak Dikenal";
    }

    // Menampilkan servo mana yang sedang digerakkan
    Serial.print("Menggerakkan ");
    Serial.print(servoName);
    Serial.println("...");

    // Membaca posisi awal servo
    currentFingerPosition = servo->read();
    Serial.print("Posisi awal servo: ");
    Serial.println(currentFingerPosition);

    do {
        int previousFingerPosition = currentFingerPosition; // Simpan posisi sebelumnya

        // Membaca posisi saat ini dari servo
        currentFingerPosition = servo->read();
        Serial.println(currentFingerPosition);

        // Menghitung error dan perubahan error
        int FingerError = finalPosition - currentFingerPosition;
        int FingerChangeInError = FingerError - errorValue;

        if (FingerError > 30) {
            FingerError = 30;
        } else if (FingerError < -30) {
            FingerError = -30;
        }

        if (FingerChangeInError > 30) {
            FingerChangeInError = 30;
        } else if (FingerChangeInError < -30) {
            FingerChangeInError = -30;
        }

        errorValue = FingerError; // Update errorValue untuk penggunaan berikutnya

        // Mengatur input fuzzy
        fuzzy->setInput(1, FingerError);
        fuzzy->setInput(2, FingerChangeInError);

        // Jalankan fuzzy inference system
        fuzzy->fuzzify();

        // Ambil output dari fuzzy inference
        float fuzzyOutput = fuzzy->defuzzify(1);

        // Menghitung posisi baru
        int newFingerPosition = currentFingerPosition + fuzzyOutput;

        // Pastikan posisi baru tidak melebihi batas servo
        newFingerPosition = constrain(newFingerPosition, 1, 180);

        // Gerakkan servo ke posisi baru
        servo->write(newFingerPosition);

        // Delay untuk stabilitas gerakan servo
        delay(15);
        
        // Periksa apakah waktu yang telah berlalu lebih dari 2 detik (2000 ms)
        if (millis() - fingerStartTime > 500) {
            Serial.println("Waktu maksimum iterasi tercapai.");
            break;
        }

    } while (abs(finalPosition - currentFingerPosition) > 2);

    unsigned long fingerElapsedTime = millis() - fingerStartTime;
    Serial.print("Time to reach finger target position: ");
    Serial.println(fingerElapsedTime);
}

void addFuzzyRule(Fuzzy *fuzzy, int ruleID, FuzzySet *input1, FuzzySet *input2, FuzzySet *output) {
    FuzzyRuleAntecedent *antecedent = new FuzzyRuleAntecedent();
    antecedent->joinWithAND(input1, input2);
    FuzzyRuleConsequent *consequent = new FuzzyRuleConsequent();
    consequent->addOutput(output);
    fuzzy->addFuzzyRule(new FuzzyRule(ruleID, antecedent, consequent));
}

// Inisialisasi komunikasi serial
void setup() {
    Serial.begin(115200);
    Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
    Serial.println("Memulai pengumpulan data...");

    // Inisialisasi model
    tf.setNumInputs(1 * 12 * 1);
    tf.setNumOutputs(10);

    tf.resolver.AddConv2D();
    tf.resolver.AddMaxPool2D();
    tf.resolver.AddL2Normalization();
    tf.resolver.AddReshape();
    tf.resolver.AddRelu();
    tf.resolver.AddFullyConnected();
    tf.resolver.AddSoftmax();
    tf.resolver.AddMul();
    tf.resolver.AddAdd();
    tf.resolver.AddExpandDims();

    while (!tf.begin(irisModel).isOk()) {
        Serial.println(tf.exception.toString());
    }

    // Attach servo ke pin yang sesuai
    servoJempol.attach(servoPins[0], 544, 2400);
    servoTelunjuk.attach(servoPins[1], 544, 2400);
    servoTengah.attach(servoPins[2]);
    servoManis.attach(servoPins[3], 544, 2500);
    servoKelingking.attach(servoPins[4], 544, 2500);

    // Set posisi awal servo
    for (int i = 0; i < 5; i++) {
      Servo* currentServo = getServo(i);
      currentServo->write(pos);
    }

    // FuzzyInput untuk error dengan nama yang diubah
    FuzzyInput *error = new FuzzyInput(1);
    error->addFuzzySet(eNB);
    error->addFuzzySet(eNM);
    error->addFuzzySet(eNS);
    error->addFuzzySet(eZE);
    error->addFuzzySet(ePS);
    error->addFuzzySet(ePM);
    error->addFuzzySet(ePB);
    fuzzy->addFuzzyInput(error);

    // Tambahkan fuzzy set ke fuzzy input change in error
    FuzzyInput *changeInError = new FuzzyInput(2);
    changeInError->addFuzzySet(cNB);
    changeInError->addFuzzySet(cNM);
    changeInError->addFuzzySet(cNS);
    changeInError->addFuzzySet(cZE);
    changeInError->addFuzzySet(cPS);
    changeInError->addFuzzySet(cPM);
    changeInError->addFuzzySet(cPB);
    fuzzy->addFuzzyInput(changeInError);

    // Tambahkan fuzzy set ke fuzzy output
    FuzzyOutput *output = new FuzzyOutput(1);
    output->addFuzzySet(outNB);
    output->addFuzzySet(outNM);
    output->addFuzzySet(outNS);
    output->addFuzzySet(outZE);
    output->addFuzzySet(outPS);
    output->addFuzzySet(outPM);
    output->addFuzzySet(outPB);
    fuzzy->addFuzzyOutput(output);

    addFuzzyRule(fuzzy, 1, eNB, cNB, outNB);
    addFuzzyRule(fuzzy, 2, eNB, cNS, outNB);
    addFuzzyRule(fuzzy, 3, eNB, cZE, outNB);
    addFuzzyRule(fuzzy, 4, eNB, cPS, outNS);
    addFuzzyRule(fuzzy, 5, eNB, cPB, outZE);

    addFuzzyRule(fuzzy, 6, eNS, cNB, outNB);
    addFuzzyRule(fuzzy, 7, eNS, cNS, outNB);
    addFuzzyRule(fuzzy, 8, eNS, cZE, outNS);
    addFuzzyRule(fuzzy, 9, eNS, cPS, outZE);
    addFuzzyRule(fuzzy, 10, eNS, cPB, outPS);

    addFuzzyRule(fuzzy, 11, eZE, cNB, outNB);
    addFuzzyRule(fuzzy, 12, eZE, cNS, outNS);
    addFuzzyRule(fuzzy, 13, eZE, cZE, outZE);
    addFuzzyRule(fuzzy, 14, eZE, cPS, outPS);
    addFuzzyRule(fuzzy, 15, eZE, cPB, outPB);

    addFuzzyRule(fuzzy, 16, ePS, cNB, outNS);
    addFuzzyRule(fuzzy, 17, ePS, cNS, outZE);
    addFuzzyRule(fuzzy, 18, ePS, cZE, outPS);
    addFuzzyRule(fuzzy, 19, ePS, cPS, outPB);
    addFuzzyRule(fuzzy, 20, ePS, cPB, outPB);

    addFuzzyRule(fuzzy, 21, ePB, cNB, outZE);
    addFuzzyRule(fuzzy, 22, ePB, cNS, outPS);
    addFuzzyRule(fuzzy, 23, ePB, cZE, outPB);
    addFuzzyRule(fuzzy, 24, ePB, cPS, outPB);
    addFuzzyRule(fuzzy, 25, ePB, cPB, outPB);
}

void loop() {
  int predictedLabel = 0;

    // Cek apakah ada data yang masuk dari perangkat
    if (Serial2.available() >= numChannels) {
              for (int i = 0; i < numChannels; i++) {
            if (dataCount[i] < maxSets) {
                int data = Serial2.read();
                channelData[i][dataCount[i]] = data;
                dataCount[i]++;
            }
        }

        // Jika semua channel sudah memiliki 30 data, hitung fitur dan rata-rata
        bool allChannelsFull = true;
        for (int i = 0; i < numChannels; i++) {
            if (dataCount[i] < maxSets) {
                allChannelsFull = false;
                break;
            }
        }

        if (allChannelsFull) {
            float avgMAV = 0;
            float avgRMS = 0;
            float avgWL = 0;
            float avgAmpFirstBurst = 0;

            for (int i = 0; i < numChannels; i++) {
                mavValues[i] = calculateMAV(channelData[i], maxSets);
                rmsValues[i] = calculateRMS(channelData[i], maxSets);
                wlValues[i] = calculateWaveformLength(channelData[i], maxSets);
                amplitudeFirstBurst[i] = calculateAmplitudeFirstBurst(channelData[i], maxSets, 0.1);

                avgMAV += mavValues[i];
                avgRMS += rmsValues[i];
                avgWL += wlValues[i];
                avgAmpFirstBurst += amplitudeFirstBurst[i];

                dataCount[i] = 0;  // Reset count
            }

            avgMAV /= numChannels;
            avgRMS /= numChannels;
            avgWL /= numChannels;
            avgAmpFirstBurst /= numChannels;

            float x1[12];
            
            // Set nilai x1[0] menjadi 172.51897658120
            x1[0] = 172.51897658120;
            
            for (int i = 1; i < 8; i++) {
                x1[i] = channelData[i][maxSets - 1];  // Mengambil nilai terakhir yang valid
            }

            // Masukkan nilai rata-rata sebagai fitur 9-12
            x1[8] = avgWL;
            x1[9] = avgRMS;
            x1[10] = avgMAV;
            x1[11] = avgAmpFirstBurst;

            Serial.println("12 Data Input untuk CNN:");
            for (int i = 0; i < 12; i++) {
                Serial.print("x1["); Serial.print(i); Serial.print("]: ");
                Serial.println(x1[i]);
            }
            Serial.println("");

            float normalizedInput[12];
            for (int i = 0; i < 8; i++) {
                normalizedInput[i] = (float)((x1[i] / 255.0) * 255);
            }

            normalizedInput[8] = normalizeTo255(x1[8], min_wl, max_wl);
            normalizedInput[9] = normalizeTo255(x1[9], min_rms, max_rms);
            normalizedInput[10] = normalizeTo255(x1[10], min_mav, max_mav);
            normalizedInput[11] = normalizeTo255(x1[11], min_amp, max_amp);

            Serial.println("Data Setelah Normalisasi:");
            for (int i = 0; i < 12; i++) {
                Serial.print("x1["); Serial.print(i); Serial.print("]: ");
                Serial.println(normalizedInput[i]);
            }
            Serial.println("");

            if (!tf.predict((float*)normalizedInput).isOk()) {
                Serial.println(tf.exception.toString());
                return;
            }

            Serial.println("Probabilities for each class:");
            for (uint16_t i = 0; i < tf.numOutputs; i++) {
                Serial.print("Class ");
                Serial.print(i);
                Serial.print(": ");
                Serial.println(tf.output(i), 4);
            }

            Serial.print("Predicted class: ");
            int predictedLabel = tf.classification;
            Serial.println(predictedLabel);
            Serial.print("Time for prediction: ");
            Serial.print(tf.benchmark.microseconds());
            Serial.println(" us");
            Serial.println("");

            if (predictedLabel == 0) {
              Serial.println("Open");
              unsigned long startTime = millis();
              moveServoWithFuzzy(&servoJempol, 0);
              moveServoWithFuzzy(&servoTelunjuk, 0);
              moveServoWithFuzzy(&servoTengah, 0);
              moveServoWithFuzzy(&servoManis, 0);
              moveServoWithFuzzy(&servoKelingking, 0);
              unsigned long elapsedTime = millis() - startTime;
              Serial.print("Time to reach target position: ");
              Serial.println(elapsedTime);
              Serial.println("Open");
              delay(15);
            }

            if (predictedLabel == 1) {
              Serial.println("Close"); 
              unsigned long startTime = millis();
              moveServoWithFuzzy(&servoJempol, 180);
              moveServoWithFuzzy(&servoTelunjuk, 180);
              moveServoWithFuzzy(&servoTengah, 180);
              moveServoWithFuzzy(&servoManis, 180);
              moveServoWithFuzzy(&servoKelingking, 180);
              unsigned long elapsedTime = millis() - startTime;
              Serial.print("Time to reach target position: ");
              Serial.println(elapsedTime);
              Serial.println("Close"); 
              delay(15);
            }

            if (predictedLabel == 2) {
              Serial.println("Finegrip"); 
              unsigned long startTime = millis();
              moveServoWithFuzzy(&servoJempol, 180);
              moveServoWithFuzzy(&servoTelunjuk, 145);
              moveServoWithFuzzy(&servoTengah, 0);
              moveServoWithFuzzy(&servoManis, 0);
              moveServoWithFuzzy(&servoKelingking, 0);
              unsigned long elapsedTime = millis() - startTime;
              Serial.print("Time to reach target position: ");
              Serial.println(elapsedTime);
              Serial.println("Finegrip");
              delay(15);
            }

            if (predictedLabel == 3) {
              Serial.println("Pointer"); 
              unsigned long startTime = millis();
              moveServoWithFuzzy(&servoJempol, 180);
              moveServoWithFuzzy(&servoTelunjuk, 0);
              moveServoWithFuzzy(&servoTengah, 180);
              moveServoWithFuzzy(&servoManis, 180);
              moveServoWithFuzzy(&servoKelingking, 160);
              unsigned long elapsedTime = millis() - startTime;
              Serial.print("Time to reach target position: ");
              Serial.println(elapsedTime);
              Serial.println("Pointer");
              delay(15);
            }

            if (predictedLabel == 4) {
              Serial.println("Agree"); 
              unsigned long startTime = millis();
              moveServoWithFuzzy(&servoJempol, 0);
              moveServoWithFuzzy(&servoTelunjuk, 160);
              moveServoWithFuzzy(&servoTengah, 180);
              moveServoWithFuzzy(&servoManis, 180);
              moveServoWithFuzzy(&servoKelingking, 160);
              unsigned long elapsedTime = millis() - startTime;
              Serial.print("Time to reach target position: ");
              Serial.println(elapsedTime);
              Serial.println("Agree"); 
              delay(15);
            }

            if (predictedLabel == 5) {
              Serial.println("Two"); 
              unsigned long startTime = millis();
              moveServoWithFuzzy(&servoJempol, 180);
              moveServoWithFuzzy(&servoTelunjuk, 0);
              moveServoWithFuzzy(&servoTengah, 0);
              moveServoWithFuzzy(&servoManis, 180);
              moveServoWithFuzzy(&servoKelingking, 160);
              unsigned long elapsedTime = millis() - startTime;
              Serial.print("Time to reach target position: ");
              Serial.println(elapsedTime);
              Serial.println("Two");
              delay(15);
            }

            if (predictedLabel == 6) {
              Serial.println("Three"); 
              unsigned long startTime = millis();
              moveServoWithFuzzy(&servoJempol, 180);
              moveServoWithFuzzy(&servoTelunjuk, 0);
              moveServoWithFuzzy(&servoTengah, 0);
              moveServoWithFuzzy(&servoManis, 0);
              moveServoWithFuzzy(&servoKelingking, 160);
              unsigned long elapsedTime = millis() - startTime;
              Serial.print("Time to reach target position: ");
              Serial.println(elapsedTime);
              Serial.println("Three"); 
              delay(15);
            }

            if (predictedLabel == 7) {
              Serial.println("Four"); 
              unsigned long startTime = millis();
              moveServoWithFuzzy(&servoJempol, 180);
              moveServoWithFuzzy(&servoTelunjuk, 0);
              moveServoWithFuzzy(&servoTengah, 0);
              moveServoWithFuzzy(&servoManis, 0);
              moveServoWithFuzzy(&servoKelingking, 0);
              unsigned long elapsedTime = millis() - startTime;
              Serial.print("Time to reach target position: ");
              Serial.println(elapsedTime);
              Serial.println("Four"); 
              delay(15);
            }

            if (predictedLabel == 8) {
              Serial.println("Pinch"); 
              unsigned long startTime = millis();
              moveServoWithFuzzy(&servoJempol, 160);
              moveServoWithFuzzy(&servoTelunjuk, 140);
              moveServoWithFuzzy(&servoTengah, 100);
              moveServoWithFuzzy(&servoManis, 180);
              moveServoWithFuzzy(&servoKelingking, 160);
              unsigned long elapsedTime = millis() - startTime;
              Serial.print("Time to reach target position: ");
              Serial.println(elapsedTime);
              Serial.println("Pinch"); 
              delay(15);
            }

            if (predictedLabel == 9) {
              Serial.println("Halfclose"); 
              unsigned long startTime = millis();
              moveServoWithFuzzy(&servoJempol, 160);
              moveServoWithFuzzy(&servoTelunjuk, 140);
              moveServoWithFuzzy(&servoTengah, 110);
              moveServoWithFuzzy(&servoManis, 180);
              moveServoWithFuzzy(&servoKelingking, 120);
              unsigned long elapsedTime = millis() - startTime;
              Serial.print("Time to reach target position: ");
              Serial.println(elapsedTime);
              Serial.println("Halfclose");
              delay(15);
            }
        }        
    }    
    delay(10); // Penundaan kecil
}

