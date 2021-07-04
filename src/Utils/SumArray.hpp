#ifndef SUM_ARRAY
#define SUM_ARRAY

template<typename T, int capacity>
struct sum_array {
    T arr[capacity];
    T sum;
    int current;

    void push(T elem) {
        if(current >= capacity) {
            current = 0;
        }
        sum += (elem - arr[current]);
        arr[current] = elem;
        current++;
    }

    void reset() {
        sum = (T) 0;
        current = 0;
        for(int i=0; i < capacity; i++) {
            arr[i] = (T) 0;
        }
    }
};

#endif