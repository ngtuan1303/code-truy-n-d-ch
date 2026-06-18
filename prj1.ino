#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <HX711.h>

// ==============================================================================
// 1. CẤU HÌNH PHẦN CỨNG & HẰNG SỐ
// ==============================================================================

const int CHAN_HX711_DT      = 19;
const int CHAN_HX711_SCK     = 18;
const int CHAN_CAM_BIEN_GIOT = 4;

const char* TEN_WIFI      = "QA 5G";
const char* MAT_KHAU_WIFI = "qa27032000";

// Server tự xử lý heartbeat + update trong 1 endpoint duy nhất
const char* URL_SERVER = "http://192.168.1.6:8000/api/du-lieu-esp";

// MAC Address của thiết bị này — dùng để server phân biệt nhiều ESP32
// Thay bằng MAC thực của board (xem Serial Monitor khi chạy setup)
const char* MAC_ADDRESS = "AA:BB:CC:DD:EE:22";

// --- Chu kỳ non-blocking ---
const unsigned long CHU_KY_DOC_LOADCELL  = 300;   // Đọc loadcell mỗi 300ms
const unsigned long CHU_KY_TINH_TOAN     = 1000;  // Tính toán mỗi 1 giây
const unsigned long CHU_KY_CAP_NHAT_LCD  = 1000;  // Cập nhật LCD mỗi 1 giây
const unsigned long CHU_KY_GUI_SERVER    = 5000;  // Gửi dữ liệu mỗi 5 giây

// --- Ngưỡng ---
const unsigned long THOI_GIAN_DEBOUNCE_GIOT  = 120;
const unsigned long NGUONG_CANH_BAO_MAT_GIOT = 15000;

// --- Cấu hình Loadcell ---
const float HE_SO_HIEU_CHUAN_HX711 = -55.448; // Thay bằng hệ số thực tế của bạn

// Số mẫu lấy trung bình mỗi lần đọc HX711.
// Tăng lên giảm nhiễu nhưng chậm hơn (5–10 là hợp lý).
const int SO_MAU_TRUNG_BINH = 8;

// Ngưỡng thay đổi tối thiểu để cập nhật khối lượng.
// Lọc rung động nhỏ < 1g, tránh LCD nhảy liên tục.
const float NGUONG_THAY_DOI_GRAM = 1.0;

// ==============================================================================
// 2. BIẾN TOÀN CỤC
// ==============================================================================

// --- Bộ định thời ---
unsigned long thoi_gian_doc_loadcell_truoc = 0;
unsigned long thoi_gian_tinh_toan_truoc    = 0;
unsigned long thoi_gian_cap_nhat_lcd_truoc = 0;
unsigned long thoi_gian_gui_server_truoc   = 0;

// --- Dữ liệu Loadcell ---
float khoi_luong_hien_tai  = 0.0;
float khoi_luong_con_lai   = 0.0;
float khoi_luong_truoc_do  = 0.0;

// --- Dữ liệu đếm giọt ---
volatile unsigned long tong_so_giot        = 0;
volatile unsigned long thoi_gian_giot_cuoi = 0;
unsigned long          tong_so_giot_truoc_do = 0;

// --- Tính toán ---
float toc_do_truyen_ml_phut = 0.0;
int   giot_moi_phut         = 0;

// --- Trạng thái ---

bool    canh_bao_tac_nghen  = false;

// --- Đối tượng ---
HX711             loadcell;
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ==============================================================================
// 3. ISR ĐẾM GIỌT
// ==============================================================================

void IRAM_ATTR xu_ly_ngat_dem_giot() {
    unsigned long t = millis();
    if (t - thoi_gian_giot_cuoi > THOI_GIAN_DEBOUNCE_GIOT) {
        tong_so_giot++;
        thoi_gian_giot_cuoi = t;
    }
}

// ==============================================================================
// 4. SETUP
// ==============================================================================

void setup() {
    Serial.begin(115200);
    Serial.println("\n[SYSTEM] Khoi dong He thong Truyen dich Thong minh");

    // LCD
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Dang khoi dong..");

    // Cảm biến giọt
    pinMode(CHAN_CAM_BIEN_GIOT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(CHAN_CAM_BIEN_GIOT), xu_ly_ngat_dem_giot, FALLING);

    // Loadcell
    loadcell.begin(CHAN_HX711_DT, CHAN_HX711_SCK);
    loadcell.set_scale(HE_SO_HIEU_CHUAN_HX711);

    // Tare: lấy trung bình 20 mẫu để zero chính xác
    Serial.println("[LOADCELL] Dang tare (lay 20 mau)...");
    loadcell.tare(20);
    Serial.println("[LOADCELL] Tare xong.");

    // Đọc khối lượng ban đầu (lấy SO_MAU_TRUNG_BINH mẫu)
    if (loadcell.is_ready()) {
        khoi_luong_hien_tai = loadcell.get_units(SO_MAU_TRUNG_BINH);
        if (khoi_luong_hien_tai < 0) khoi_luong_hien_tai = 0;
        khoi_luong_con_lai  = khoi_luong_hien_tai;
        khoi_luong_truoc_do = khoi_luong_hien_tai;
        Serial.printf("[LOADCELL] Khoi luong ban dau: %.1f g\n", khoi_luong_hien_tai);
    }

    // WiFi
    WiFi.begin(TEN_WIFI, MAT_KHAU_WIFI);
    Serial.print("[WIFI] Dang ket noi");
    unsigned long t_wifi = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t_wifi < 10000) {
        delay(500); Serial.print(".");
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("[WIFI] Ket noi thanh cong! IP: ");
        Serial.println(WiFi.localIP());
        // In MAC address thực của board để cập nhật vào MAC_ADDRESS nếu cần
        Serial.print("[WIFI] MAC Address thuc: ");
        Serial.println(WiFi.macAddress());
    } else {
        Serial.println("[WIFI] Khong ket noi duoc, chay offline.");
    }

    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("He thong san sang");
}

// ==============================================================================
// 5. MODULE: ĐỌC CẢM BIẾN

void xu_ly_doc_cam_bien() {
    if (millis() - thoi_gian_doc_loadcell_truoc < CHU_KY_DOC_LOADCELL) return;
    thoi_gian_doc_loadcell_truoc = millis();

    if (!loadcell.is_ready()) {
        Serial.println("[ERROR] Mat ket noi Loadcell HX711!");
        return;
    }

    // Lấy trung bình SO_MAU_TRUNG_BINH mẫu phần cứng — giảm nhiễu tốt nhất
    float gia_tri_doc = loadcell.get_units(SO_MAU_TRUNG_BINH);
    if (gia_tri_doc < 0) gia_tri_doc = 0;

    // Chỉ cập nhật khi chênh lệch vượt ngưỡng tối thiểu.
    // Tránh LCD và server nhận giá trị "nhảy" do nhiễu điện/cơ học.
    if (fabs(gia_tri_doc - khoi_luong_hien_tai) >= NGUONG_THAY_DOI_GRAM) {
        khoi_luong_hien_tai = gia_tri_doc;
        khoi_luong_con_lai  = khoi_luong_hien_tai;
    }
}

// ==============================================================================
// 6. MODULE: TÍNH TOÁN
// ==============================================================================

void xu_ly_tinh_toan() {
    if (millis() - thoi_gian_tinh_toan_truoc < CHU_KY_TINH_TOAN) return;

    float delta_t_s = (millis() - thoi_gian_tinh_toan_truoc) / 1000.0;
    thoi_gian_tinh_toan_truoc = millis();

    // Đọc snapshot biến volatile an toàn (tắt ngắt tạm thời)
    noInterrupts();
    unsigned long giot_hien_tai = tong_so_giot;
    interrupts();

    // Tính giọt/phút trong chu kỳ 1 giây
    unsigned long giot_trong_chu_ky = giot_hien_tai - tong_so_giot_truoc_do;
    giot_moi_phut = (int)((giot_trong_chu_ky / delta_t_s) * 60.0);
    tong_so_giot_truoc_do = giot_hien_tai;

    // Tính tốc độ từ chênh lệch khối lượng
    float chenh_lech = khoi_luong_truoc_do - khoi_luong_con_lai;
    if (chenh_lech > 0) {
        toc_do_truyen_ml_phut = (chenh_lech / delta_t_s) * 60.0;
    } else {
        toc_do_truyen_ml_phut = 0;
    }
    khoi_luong_truoc_do = khoi_luong_con_lai;

    Serial.printf("[SENSOR] KL: %.1fg | Toc do: %.1fml/ph | Tong giot: %lu | %d giot/ph\n",
                  khoi_luong_con_lai, toc_do_truyen_ml_phut, giot_hien_tai, giot_moi_phut);
}

// ==============================================================================
// 7. MODULE: HIỂN THỊ LCD

void xu_ly_hien_thi() {
    if (millis() - thoi_gian_cap_nhat_lcd_truoc < CHU_KY_CAP_NHAT_LCD) return;
    thoi_gian_cap_nhat_lcd_truoc = millis();

    char dong_1[17];
    char dong_2[17];

    // Dòng 1: Tốc độ giọt (giọt/phút)
    snprintf(dong_1, sizeof(dong_1), "Toc do:%4d g/ph", giot_moi_phut);

    // Dòng 2: Khối lượng dịch còn lại (gram)
    snprintf(dong_2, sizeof(dong_2), "Con lai:%6.1fg  ", khoi_luong_con_lai);

    lcd.setCursor(0, 0); lcd.print(dong_1);
    lcd.setCursor(0, 1); lcd.print(dong_2);
}

// ==============================================================================
// 8. MODULE: GỬI WIFI


void xu_ly_wifi() {
    if (millis() - thoi_gian_gui_server_truoc < CHU_KY_GUI_SERVER) return;
    thoi_gian_gui_server_truoc = millis();

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WIFI] Mat ket noi mang!");
        return;
    }

    // Đóng gói JSON đúng cấu trúc backend yêu cầu
    char payload[128];
    snprintf(payload, sizeof(payload),
        "{\"mac_address\":\"%s\",\"current_drop_rate\":%.1f,\"current_weight\":%.1f}",
        MAC_ADDRESS,
        toc_do_truyen_ml_phut,   // current_drop_rate = giọt/phút
        khoi_luong_con_lai       // current_weight = khối lượng hiện tại (gồm vỏ)
    );

    HTTPClient http;
    http.begin(URL_SERVER);
    http.addHeader("Content-Type", "application/json");

    int ma = http.POST(payload);

    if (ma > 0) {
        Serial.printf("[WIFI] Gui thanh cong (HTTP %d) | %s\n", ma, payload);
    } else {
        Serial.printf("[WIFI] Loi POST: %s\n", http.errorToString(ma).c_str());
    }

    http.end();
}

// ==============================================================================
// 10. MODULE: CẢNH BÁO
// ==============================================================================

void xu_ly_canh_bao() {
    if (tong_so_giot > 0 &&
        (millis() - thoi_gian_giot_cuoi > NGUONG_CANH_BAO_MAT_GIOT)) {
        if (!canh_bao_tac_nghen) {
            canh_bao_tac_nghen = true;
            Serial.println("[ALARM] Khong co giot chay qua! Tac nghen hoac het dich.");
        }
    } else {
        canh_bao_tac_nghen = false;
    }
}

// ==============================================================================
// 11. LOOP CHÍNH
// ==============================================================================

void loop() {
    xu_ly_doc_cam_bien();   
    xu_ly_tinh_toan();      
    xu_ly_hien_thi();      
    xu_ly_wifi();           
    xu_ly_canh_bao();       
}
