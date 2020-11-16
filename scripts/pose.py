class Pose:
    def __init__(self, x, y):
        '''
        Initialization function. 
        Contains the x and y-coordinates in separate instance variables
        '''
        self.x = x
        self.y = y
    
    def __str__(self):
        '''String method for testing purposes'''
        return 'Pose(' + str(self.x) + ' , ' + str(self.y) + ')'
