#ifndef FILTER_HPP
#define FILTER_HPP

template<typename T>
class Filter
{
public:

        Filter() {}

        void set_decade_attenuation(float decade_attenuation)
        {
            this->decade_attenuation = decade_attenuation;
        }

        float filter(T speed)
        {
            if(speed <= this->cut_off_speed || speed > 2*this->cut_off_speed) return 1.0f;
            return 1 + this->grade * (speed - this->cut_off_speed);
        }

        void reset_filter(T target_speed)
        {
            this->cut_off_speed = 1.05*target_speed;
            this->grade = -this->decade_attenuation / (0.1f * (float) target_speed);
        }

private:
        T cut_off_speed;
        float decade_attenuation;
        float grade;
};


#endif