#include "IMU_HUST.h"

IMU_HUST::IMU_HUST(TwoWire &i2c)
    :_i2c(&i2c),_accCoef(0.03f),_gyroCoef(0.97f)  {}

void IMU_HUST::init(){
    writeToIMU(MPU6050_SMPLRT_DIV, 0x00);//Đặt thanh ghi chia tỷ lệ lấy mẫu (SMPLRT_DIV) thành 0x00, tức là tỷlệ lấy mẫu không được chia và tỷ lệ lấy mẫu của đầu ra cảm biến là tỷ lệ đầu ra của con quay hồi chuyển (thường là 8kHz)
    writeToIMU(MPU6050_CONFIG, 0x00);//Đặt thanh ghi cấu hình (CONFIG) thành 0x00, tức là tắt bộ lọc thấp
    writeToIMU(MPU6050_GYRO_CONFIG, 0x08);//Đặt thanh ghi cấu hình con quay (GYRO_CONFIG) thành 0x08, tức là đặt dải con quay thành ±500 độ/giây
    writeToIMU(MPU6050_ACCEL_CONFIG, 0x00);//Đặt thanh ghi cấu hình gia tốc (ACCEL_CONFIG) thành 0x00, tức là đặt dải gia tốc thành ±2g
    writeToIMU(MPU6050_PWR_MGMT_1, 0x01);//Đặt thanh ghi quản lý nguồn (PWR_MGMT_1) thành 0x01, tức là đặt cảm biến vào chế độ làm việc và chọn nguồn thời gian bên trong.
    calGyroOffsets();//Mặc định, khởi tạo sẽ được hiệu chuẩn
    update();
    _preInterval = millis();
    angleGyro[0] = 0;
    angleGyro[1] = 0;
    angle[0] = angleAcc[0];
    angle[1] = angleAcc[1];

    
}

void IMU_HUST::calGyroOffsets(){
    float x = 0, y = 0, z = 0;
    int16_t rx, ry, rz;

    delay(1000);
    Serial.println();
    Serial.println("========================================");
    Serial.println("Calculating gyro offsets");
    Serial.print("DO NOT MOVE MPU6050");

    for (int i = 0; i < 3000; i++) {
        if (i % 1000 == 0) Serial.print(".");
        
        _i2c->beginTransmission(MPU6050_ADDR);
        _i2c->write(0x43);
        _i2c->endTransmission(false);
        _i2c->requestFrom((int)MPU6050_ADDR, 6);

        rx = _i2c->read() << 8 | _i2c->read();
        ry = _i2c->read() << 8 | _i2c->read();
        rz = _i2c->read() << 8 | _i2c->read();

        x += ((float)rx) / 65.5;
        y += ((float)ry) / 65.5;
        z += ((float)rz) / 65.5;
    }

    _gyroOffset[0] = x / 3000;
    _gyroOffset[1] = y / 3000;
    _gyroOffset[2] = z / 3000;

    Serial.println("Done!");
    Serial.printf("%f,%f,%f\n",_gyroOffset[0],_gyroOffset[1],_gyroOffset[2]);
    Serial.print("========================================");

    delay(1000);
}


void IMU_HUST::update(){
    _i2c->beginTransmission(MPU6050_ADDR);
    _i2c->write(0x3B);
    _i2c->endTransmission(false);
    _i2c->requestFrom((int)MPU6050_ADDR, 14);//Đọc dữ liệu từ cảm biến MPU6050, bao gồm gia tốc, nhiệt độ và con quay, và lưu trữ chúng trong bộ nhớ I²C

    //Lấy dữ liệu gốc từ thanh ghi
    int16_t rawAccX = _i2c->read() << 8 | _i2c->read();
    int16_t rawAccY = _i2c->read() << 8 | _i2c->read();
    int16_t rawAccZ = _i2c->read() << 8 | _i2c->read();
    int16_t rawTemp = _i2c->read() << 8 | _i2c->read();
    int16_t rawGyroX = _i2c->read() << 8 | _i2c->read();
    int16_t rawGyroY = _i2c->read() << 8 | _i2c->read();
    int16_t rawGyroZ = _i2c->read() << 8 | _i2c->read();

    //Tính toán nhiệt độ
    temp = (rawTemp + 12412.0) / 340.0;

    //Tính toán gia tốc 16384.0 vì gia tốc cảm biến có dải là ±2g, và độ phân giải là 16 bit.
    acc[0] = ((float)rawAccX) / 16384.0;
    acc[1] = ((float)rawAccY) / 16384.0;
    acc[2] = ((float)rawAccZ) / 16384.0;

    //Tính toán độ nghiêng của hai trục bằng cách sử dụng dữ liệu gia tốc kế

    angleAcc[0] = atan2(acc[1], acc[2] + abs(acc[0])) * 360 / 2.0 / M_PI;
    angleAcc[1] = atan2(acc[0], acc[2] + abs(acc[1])) * 360 / -2.0 / M_PI;

//Giá trị được chia cho 65,5 vì con quay hồi chuyển có phạm vi ±500 độ/giây, tương ứng với độ phân giải 16 bit.
// Độ lệch con quay hồi chuyển sau đó được trừ đi để loại bỏ độ trôi bằng 0    
    gyro[0] = ((float)rawGyroX) / 65.5 - _gyroOffset[0];
    gyro[1] = ((float)rawGyroY) / 65.5 - _gyroOffset[1];
    gyro[2] = ((float)rawGyroZ) / 65.5 - _gyroOffset[2];

    //Thời gian khoảng ms
    _interval = (millis() - _preInterval) * 0.001;

//Cập nhật giá trị góc theo vận tốc góc và khoảng thời gian của con quay hồi chuyển.
    angleGyro[0] += gyro[0] * _interval;
    angleGyro[1] += gyro[1] * _interval;
    angleGyro[2] += gyro[2] * _interval;

    //Tích hợp góc gia tốc và con quay hồi chuyển
    //Sử dụng trung bình có trọng số (thuật toán hợp nhất) để kết hợp các góc từ con quay hồi chuyển và gia tốc để thu được giá trị góc chính xác và ổn định hơn. Trong đó, gyroCoef và accCoef là hai hệ số trọng số, biểu thị độ tin cậy của dữ liệu con quay hồi chuyển và gia tốc.
    angle[0] = (_gyroCoef * (angle[0] + gyro[0] * _interval)) + (_accCoef * angleAcc[0]);
    angle[1] = (_gyroCoef * (angle[1] + gyro[1] * _interval)) + (_accCoef * angleAcc[1]);
    angle[2] = angleGyro[2];

    //Cập nhật thời gian cập nhật trước
    _preInterval = millis();
}


void IMU_HUST::writeToIMU(uint8_t addr, uint8_t data){
    _i2c->beginTransmission(MPU6050_ADDR);
    _i2c->write(addr);
    _i2c->write(data);
    _i2c->endTransmission();
}

uint8_t IMU_HUST::readFromIMU(uint8_t addr){
    _i2c->beginTransmission(MPU6050_ADDR);
    _i2c->write(addr);
    _i2c->endTransmission();

    _i2c->requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)1);
    return _i2c->read();
}

void IMU_HUST::setCoef(float accCoef, float gyroCoef){
    _accCoef = accCoef;
    _gyroCoef = gyroCoef;
}

    void IMU_HUST::setGyroOffsets(float x, float y, float z){
    _gyroOffset[0] = x;
    _gyroOffset[1] = y;
    _gyroOffset[2] = z;
}