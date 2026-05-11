#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <HX711.h>

// ==============================================================================
// 1. CẤU HÌNH PHẦN CỨNG & HẰNG SỐ (CONFIGURATION & CONSTANTS)
// ==============================================================================

// --- Cấu hình chân (Pins) ---
const int CHAN_HX711_DT       = 19;
const int CHAN_HX711_SCK      = 18;
const int CHAN_CAM_BIEN_GIOT  = 4;
// I2C mặc định của ESP32: SDA = 21, SCL = 22

// --- Cấu hình Mạng & Server ---
const char* TEN_WIFI          = "XMen 2";
const char* MAT_KHAU_WIFI     = "0915610611";

// Đã tách thành 2 URL riêng biệt theo yêu cầu của Web Server
const char* URL_REGISTER      = "https://bme-1.vercel.app/api/esp32/register";
const char* URL_UPDATE        = "https://bme-1.vercel.app/api/esp32/update";

// --- Cấu hình Chu kỳ chạy (ms) - Kiến trúc Non-blocking ---
const unsigned long CHU_KY_DOC_LOADCELL = 500;
const unsigned long CHU_KY_TINH_TOAN    = 1000;
const unsigned long CHU_KY_CAP_NHAT_LCD = 3000;
const unsigned long CHU_KY_GUI_SERVER   = 5000;

// --- Cấu hình Ngưỡng (Thresholds) ---
const unsigned long THOI_GIAN_DEBOUNCE_GIOT = 120;    // Bỏ qua nhiễu < 120ms
const unsigned long NGUONG_CANH_BAO_MAT_GIOT = 15000; // Cảnh báo nếu 15s không có giọt
const float HE_SO_HIEU_CHUAN_HX711 = 420.5;          // CẦN THAY ĐỔI THEO THỰC TẾ

// ==============================================================================
// 2. BIẾN TOÀN CỤC (GLOBAL VARIABLES)
// ==============================================================================

// --- Biến Hệ thống & Thời gian (Bộ định thời) ---
unsigned long thoi_gian_doc_loadcell_truoc = 0;
unsigned long thoi_gian_tinh_toan_truoc    = 0;
unsigned long thoi_gian_cap_nhat_lcd_truoc = 0;
unsigned long thoi_gian_gui_server_truoc   = 0;

// --- Biến Dữ liệu Loadcell ---
float khoi_luong_hien_tai = 0.0;
float khoi_luong_ban_dau  = 0.0;
float khoi_luong_con_lai  = 0.0;
float khoi_luong_truoc_do = 0.0; // Dùng để tính đạo hàm (tốc độ)

// --- Biến Dữ liệu Đếm giọt ---
// Dùng 'volatile' vì biến này bị thay đổi trong Ngắt (Interrupt)
volatile unsigned long tong_so_giot        = 0;
volatile unsigned long thoi_gian_giot_cuoi = 0;
unsigned long tong_so_giot_truoc_do        = 0;

// --- Biến Tính toán ---
float toc_do_truyen_g_phut  = 0.0;
float toc_do_truyen_ml_phut = 0.0; // Giả sử 1g = 1ml đối với dịch truyền chuẩn
int giot_moi_phut           = 0;

// --- Trạng thái ---
uint8_t trang_thai_man_hinh = 0; // 0: Khối lượng, 1: Tốc độ giọt
bool canh_bao_tac_nghen     = false;

// CỜ TRẠNG THÁI ĐĂNG KÝ SERVER
bool da_dang_ky_server      = false;

// --- Đối tượng (Objects) ---
HX711 loadcell;
LiquidCrystal_I2C lcd(0x27, 16, 2); // Địa chỉ 0x27, màn 16x2

// ==============================================================================
// 3. HÀM NGẮT CHỐNG NHIỄU (INTERRUPT SERVICE ROUTINE)
// ==============================================================================

/**
 * @brief  Ngắt phần cứng khi có giọt rơi qua hồng ngoại.
 * @note   Sử dụng millis() để debounce phần mềm, chống đếm trùng 1 giọt nhiều lần.
 */
void IRAM_ATTR xu_ly_ngat_dem_giot() {
    unsigned long thoi_gian_hien_tai = millis();
    // Bỏ qua các tín hiệu quá gần nhau (nhiễu viền hoặc giọt bị vỡ)
    if (thoi_gian_hien_tai - thoi_gian_giot_cuoi > THOI_GIAN_DEBOUNCE_GIOT) {
        tong_so_giot++;
        thoi_gian_giot_cuoi = thoi_gian_hien_tai;
    }
}

// ==============================================================================
// 4. MODULE: KHỞI TẠO (SETUP)
// ==============================================================================

void setup() {
    Serial.begin(115200);
    Serial.println("\n[SYSTEM] Khoi dong He thong Truyen dich Thong minh");

    // 1. Khởi tạo LCD
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("He thong Drip...");

    // 2. Khởi tạo Cảm biến Giọt (Ngắt sườn xuống)
    pinMode(CHAN_CAM_BIEN_GIOT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(CHAN_CAM_BIEN_GIOT), xu_ly_ngat_dem_giot, FALLING);

    // 3. Khởi tạo Loadcell
    loadcell.begin(CHAN_HX711_DT, CHAN_HX711_SCK);
    loadcell.set_scale(HE_SO_HIEU_CHUAN_HX711);
    loadcell.tare(); // Reset về 0 khi chưa treo bình dịch
    
    // Đọc mẫu ban đầu để lấy khối lượng khởi điểm
    delay(1000); 
    if (loadcell.is_ready()) {
        khoi_luong_ban_dau = loadcell.get_units(10);
        khoi_luong_con_lai = khoi_luong_ban_dau;
        Serial.printf("[LOADCELL] Khoi luong ban dau: %.1f g\n", khoi_luong_ban_dau);
    }

    // 4. Khởi tạo WiFi
    WiFi.begin(TEN_WIFI, MAT_KHAU_WIFI);
    Serial.print("[WIFI] Dang ket noi");
    // Không dùng while(WiFi.status() != WL_CONNECTED) để tránh treo hệ thống cứng
    // Sẽ tự kết nối ngầm nhờ ESP32 core
}

// ==============================================================================
// 5. MODULE: XỬ LÝ NGHIỆP VỤ (BUSINESS LOGIC TASKS)
// ==============================================================================

void xu_ly_doc_cam_bien() {
    unsigned long thoi_gian_hien_tai = millis();
    if (thoi_gian_hien_tai - thoi_gian_doc_loadcell_truoc >= CHU_KY_DOC_LOADCELL) {
        thoi_gian_doc_loadcell_truoc = thoi_gian_hien_tai;

        if (loadcell.is_ready()) {
            // Đọc trung bình 3 lần để làm mượt dữ liệu
            khoi_luong_hien_tai = loadcell.get_units(3);
            
            // Loại bỏ giá trị âm do nhiễu dao động cơ học
            if (khoi_luong_hien_tai < 0) khoi_luong_hien_tai = 0;
            
            khoi_luong_con_lai = khoi_luong_hien_tai; 
        } else {
            Serial.println("[ERROR] Mat ket noi Loadcell HX711!");
        }
    }
}

void xu_ly_tinh_toan() {
    unsigned long thoi_gian_hien_tai = millis();
    if (thoi_gian_hien_tai - thoi_gian_tinh_toan_truoc >= CHU_KY_TINH_TOAN) {
        // Delta T bằng giây
        float delta_t = (thoi_gian_hien_tai - thoi_gian_tinh_toan_truoc) / 1000.0;
        thoi_gian_tinh_toan_truoc = thoi_gian_hien_tai;

        // 1. Tính số giọt/phút
        // Vô hiệu hóa ngắt tạm thời khi đọc biến volatile nhiều byte để tránh Data Race
        noInterrupts();
        unsigned long giot_hien_tai = tong_so_giot;
        interrupts();

        unsigned long giot_trong_chu_ky = giot_hien_tai - tong_so_giot_truoc_do;
        giot_moi_phut = (giot_trong_chu_ky / delta_t) * 60;
        tong_so_giot_truoc_do = giot_hien_tai;

        // 2. Tính tốc độ truyền theo khối lượng (g/phut)
        float chenh_lech_kl = khoi_luong_truoc_do - khoi_luong_con_lai;
        if (chenh_lech_kl > 0) { // Đảm bảo chỉ tính khi khối lượng giảm
            toc_do_truyen_g_phut = (chenh_lech_kl / delta_t) * 60;
            toc_do_truyen_ml_phut = toc_do_truyen_g_phut; // Giả định D = 1g/ml
        } else {
            toc_do_truyen_g_phut = 0;
            toc_do_truyen_ml_phut = 0;
        }
        khoi_luong_truoc_do = khoi_luong_con_lai;

        // In debug chuyên nghiệp
        Serial.printf("[SENSOR] KL Con lai: %.1fg | Toc do: %.1fml/m | Tong giot: %lu | %d giot/m\n", 
                      khoi_luong_con_lai, toc_do_truyen_ml_phut, giot_hien_tai, giot_moi_phut);
    }
}

void xu_ly_hien_thi() {
    unsigned long thoi_gian_hien_tai = millis();
    if (thoi_gian_hien_tai - thoi_gian_cap_nhat_lcd_truoc >= CHU_KY_CAP_NHAT_LCD) {
        thoi_gian_cap_nhat_lcd_truoc = thoi_gian_hien_tai;

        char dong_1[17];
        char dong_2[17];

        lcd.clear();
        
        // Luân phiên 2 màn hình
        if (trang_thai_man_hinh == 0) {
            snprintf(dong_1, sizeof(dong_1), "Con lai: %.1fg", khoi_luong_con_lai);
            snprintf(dong_2, sizeof(dong_2), "Toc do: %.1fml/p", toc_do_truyen_ml_phut);
        } else {
            // Lấy snapshot của giọt để tránh thay đổi trong lúc vẽ LCD
            noInterrupts();
            unsigned long giot_hien_thi = tong_so_giot;
            interrupts();
            
            snprintf(dong_1, sizeof(dong_1), "Giot/ph: %d", giot_moi_phut);
            snprintf(dong_2, sizeof(dong_2), "Tong: %lu", giot_hien_thi);
        }

        lcd.setCursor(0, 0); lcd.print(dong_1);
        lcd.setCursor(0, 1); lcd.print(dong_2);

        // Đảo trạng thái cho lần sau
        trang_thai_man_hinh = !trang_thai_man_hinh; 
    }
}

void xu_ly_wifi() {
    unsigned long thoi_gian_hien_tai = millis();
    if (thoi_gian_hien_tai - thoi_gian_gui_server_truoc >= CHU_KY_GUI_SERVER) {
        thoi_gian_gui_server_truoc = thoi_gian_hien_tai;

        if (WiFi.status() == WL_CONNECTED) {
            HTTPClient http;

            // =================================================================
            // BƯỚC 1: KIỂM TRA VÀ GỌI API REGISTER NẾU CHƯA ĐĂNG KÝ
            // =================================================================
            if (!da_dang_ky_server) {
                http.begin(URL_REGISTER);
                http.addHeader("Content-Type", "application/json");
                
                // Nếu Server yêu cầu JSON cụ thể để đăng ký, bạn sửa chuỗi này (VD: {"id":"esp32_01"})
                int ma_phan_hoi = http.POST("{\"message\":\"register_request\"}");

                if (ma_phan_hoi == HTTP_CODE_OK || ma_phan_hoi == 201 || ma_phan_hoi == 200) {
                    Serial.printf("[WIFI] Register thanh cong! Code: %d\n", ma_phan_hoi);
                    da_dang_ky_server = true; // Đánh dấu đã đăng ký xong
                } else {
                    Serial.printf("[ERROR] Register that bai. Ma loi HTTP: %d\n", ma_phan_hoi);
                }
                http.end();
                return; // Kết thúc chu kỳ này, đợi 5s sau mới bắt đầu gửi API Update
            }

            // =================================================================
            // BƯỚC 2: NẾU ĐÃ REGISTER THÀNH CÔNG -> GỌI API UPDATE DỮ LIỆU
            // =================================================================
            char payload[150];
            
            noInterrupts();
            unsigned long giot_hien_tai = tong_so_giot;
            interrupts();

            // Định dạng JSON chuẩn
            snprintf(payload, sizeof(payload), 
                "{\"khoi_luong_con_lai\":%.1f, \"toc_do_ml_phut\":%.1f, \"tong_giot\":%lu, \"giot_phut\":%d, \"canh_bao\":%d}",
                khoi_luong_con_lai, toc_do_truyen_ml_phut, giot_hien_tai, giot_moi_phut, canh_bao_tac_nghen ? 1 : 0);

            http.begin(URL_UPDATE);
            http.addHeader("Content-Type", "application/json");

            int ma_phan_hoi = http.POST(payload);

            if (ma_phan_hoi > 0) {
                Serial.printf("[WIFI] Gui UPDATE thanh cong - Code: %d\n", ma_phan_hoi);
            } else {
                Serial.printf("[ERROR] Loi gui HTTP POST Update: %s\n", http.errorToString(ma_phan_hoi).c_str());
            }
            http.end();
        } else {
             Serial.println("[WIFI] Mat ket noi mang!");
             // Nếu mất mạng và muốn bắt buộc Register lại từ đầu khi có mạng, bỏ comment dòng dưới:
             // da_dang_ky_server = false; 
        }
    }
}

void xu_ly_canh_bao() {
    unsigned long thoi_gian_hien_tai = millis();
    
    // Nếu quá lâu không có giọt mới cập nhật, coi như tắc nghẽn hoặc hết dịch
    if (tong_so_giot > 0 && (thoi_gian_hien_tai - thoi_gian_giot_cuoi > NGUONG_CANH_BAO_MAT_GIOT)) {
        if (!canh_bao_tac_nghen) {
            canh_bao_tac_nghen = true;
            Serial.println("[ALARM] CANH BAO: Khong co giot chay qua! Tac nghen hoac het dich.");
            // Tại đây có thể kích hoạt còi Buzzers
        }
    } else {
        canh_bao_tac_nghen = false;
    }
}

// ==============================================================================
// 6. MODULE CHÍNH (MAIN LOOP)
// ==============================================================================
// Vòng lặp chính gọn gàng, đóng vai trò như một Task Scheduler (Bộ điều phối tác vụ)

void loop() {
    xu_ly_doc_cam_bien();
    xu_ly_tinh_toan();
    xu_ly_hien_thi();
    xu_ly_wifi();
    xu_ly_canh_bao();
}