#include <string>
#include "esphome.h"
#include "esphome/components/sensor/sensor.h"

typedef struct
{
    float wind_dir;
    int wind_spd;
    int wind_gst;
    int temp;
    float rainfall_24h;
    float pressure;
    int dewpt;
    int humid;
} weather_data;

class AcuriteWeather : public Component, public UARTDevice
{
public:
    AcuriteWeather(UARTComponent *parent) : UARTDevice(parent),
                                            wind_dir(),
                                            wind_spd(),
                                            wind_gst(),
                                            temp(),
                                            rainfall_24h(),
                                            pressure(),
                                            dewpt(),
                                            humid()
    {
        m_update_count = RSSI_UPDATE_INTERVAL;
    }

    void setup() override
    {
        wind_dir.set_state_class(sensor::STATE_CLASS_MEASUREMENT);
        wind_spd.set_state_class(sensor::STATE_CLASS_MEASUREMENT);
        wind_gst.set_state_class(sensor::STATE_CLASS_MEASUREMENT);
        temp.set_state_class(sensor::STATE_CLASS_MEASUREMENT);
        rainfall_24h.set_state_class(sensor::STATE_CLASS_MEASUREMENT);
        pressure.set_state_class(sensor::STATE_CLASS_MEASUREMENT);
        dewpt.set_state_class(sensor::STATE_CLASS_MEASUREMENT);
        humid.set_state_class(sensor::STATE_CLASS_MEASUREMENT);
    }

    void set_sensor(Sensor &sensor, float value)
    {
        if (!sensor.has_state() || sensor.get_raw_state() != value)
            sensor.publish_state(value);
    }

    bool get_full_string(const char input, char *out, int max_len)
    {
        static int state = 0;
        static char buf[80] = {0};
        static int buf_idx = 0;

        if (buf_idx >= max_len) // Make sure we don't ever overflow the buffer
        {
            buf_idx = 0;
            state = 0;
            return false;
        }

        if (state == 0) // Search for start of measurement string
        {
            if (input == '(')
            {
                state = 1;
            }
        }
        else if (state == 1) // Read characters into the buffer
        {
            if (input != ')')
            {
                buf[buf_idx++] = input;
            }
            else
            {
                buf[buf_idx] = 0;
                strcpy(out, buf);
                buf_idx = 0;
                state = 0;
                return true;
            }
        }

        return false;
    }

    void parse_measurement_string(const char *input, weather_data *out)
    {
        int wind_dir_tmp = 0;
        int rainfall_24h_tmp = 0;
        int pressure_tmp = 0;

        sscanf(
            input,
            "%d,%d,%d,%d,%d,%d,%d,%d",
            &wind_dir_tmp,
            &out->wind_spd,
            &out->wind_gst,
            &out->temp,
            &rainfall_24h_tmp,
            &pressure_tmp,
            &out->dewpt,
            &out->humid);

        out->wind_dir = wind_dir_tmp * 0.1f;
        out->rainfall_24h = rainfall_24h_tmp * 0.01f;
        out->pressure = pressure_tmp * 0.01f;
    }

    void loop() override
    {
        const int max_line_length = 80;
        static char buffer[max_line_length];
        while (available())
        {
            if (get_full_string(read(), buffer, max_line_length) > 0)
            {
                if (m_update_count == RSSI_UPDATE_INTERVAL)
                {
                    int rssi_percent = min(max(2 * (WiFi.RSSI() + 100.0), 0.0), 100.0) * 4 / 100;
                    m_update_count = 0;
                    char rssi_tmp[10] = "";
                    sprintf(rssi_tmp, "{{%d}}", rssi_percent);
                    write_str(rssi_tmp);
                    flush();
                }
                m_update_count++;

                weather_data dat;
                parse_measurement_string(buffer, &dat);
                ESP_LOGD(
                    "acurite_weather",
                    "Wind Dir: %f Wind Speed: %d Wind (Gust): %d Temperature: %d Rainfall (24h): %f Baro Press: %f Dew Pt: %d Humid: %d",
                    dat.wind_dir,
                    dat.wind_spd,
                    dat.wind_gst,
                    dat.temp,
                    dat.rainfall_24h,
                    dat.pressure,
                    dat.dewpt,
                    dat.humid);

                set_sensor(wind_dir, dat.wind_dir);
                set_sensor(wind_spd, dat.wind_spd);
                set_sensor(wind_gst, dat.wind_gst);
                set_sensor(temp, dat.temp);
                set_sensor(rainfall_24h, dat.rainfall_24h);
                set_sensor(pressure, dat.pressure);
                set_sensor(dewpt, dat.dewpt);
                set_sensor(humid, dat.humid);
            }
        }
    }

    sensor::Sensor wind_dir;
    sensor::Sensor wind_spd;
    sensor::Sensor wind_gst;
    sensor::Sensor temp;
    sensor::Sensor rainfall_24h;
    sensor::Sensor pressure;
    sensor::Sensor dewpt;
    sensor::Sensor humid;

private:
    int m_update_count;
    const int RSSI_UPDATE_INTERVAL = 5;
};