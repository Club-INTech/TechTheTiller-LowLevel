#ifndef SUM_ARRAY
#define SUM_ARRAY

template<typename T, int capacity>
struct Integral {
    T value;
    int current;

    void add(T elem) {
        if(current >= capacity) {
            current = 0;
            value = (T) 0;
        }
        value += elem;
        current++;
    }

    void reset() {
        value = (T) 0;
        current = 0;
    }
};

#endif