class Interpolation:

    x = []
    y = []

    @classmethod
    def set_points(cls, x=[], y=[]):
        cls.x = x[:]
        cls.y = y[:]

    @classmethod
    def interpolation_output(cls, t):
        result = 0
        n = len(cls.x)
        for i in range(n):
            li = 1
            for j in range(n):
                if j != i:
                    li = li * ((t-cls.x[j]) / (cls.x[i] - cls.x[j]))
            result = result + li * cls.y[i]
        return result

    @classmethod
    def sampling(cls):
        n = len(cls.x)
        
        tmp_x = cls.x[0:n-1:int(n / 45)]
        tmp_y = cls.y[0:n-1:int(n / 45)]

        cls.x = tmp_x[:]
        cls.y = tmp_y[:]

