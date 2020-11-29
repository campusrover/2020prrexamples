import py_trees

class BaseBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(BaseBehavior, self).__init__(name)
        self.log("__init__")

    def setup(self, timeout=15):
        super(BaseBehavior, self).setup(timeout)
        self.log("setup")
        return True

    def log(self, str, vals = None):
        if (vals is  None):
            print(("%s:  " + str) % self.name)
        else:
            print(("%s:"+str+" %s") % (self.name, vals))


    
    
