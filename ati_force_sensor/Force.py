class Force:
    def __init__(self, *args):
        """
        init Force by tx,ty,tz,fx,fy,fz
        """
        if len(args) == 1:#传入一个数组
            if len(args[0]) == 6:#数组的长度为6
                self.tx = args[0][0]
                self.ty = args[0][1]
                self.tz = args[0][2]
                self.fx = args[0][3]
                self.fy = args[0][4]
                self.fz = args[0][5]
            else:
                self.set_zeros()
                # raise Error(
                #     "Could not create Force on arguments : " +
                #     '"{}"'.format(str(args))
                # )
        else:
            self.set_zeros()
            # raise Error(
            #     "Could not create Force on arguments : " +
            #     '"{}"'.format(str(args))
            # )

    def set_zeros(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.rx = 0
        self.ry = 0
        self.rz = 0

    def __repr__(self):
        return "<Force:[tx,ty,tz,fx,fy,fz]:{}>".format(
            [self.tx, self.ty, self.tz, self.fx, self.fy, self.fz]
        )

    def __copy__(self):
        """
        Copy method for creating a (deep) copy of this Transform.
        """
        return Force(self)

    def __deepcopy__(self, memo):
        return self.__copy__()
