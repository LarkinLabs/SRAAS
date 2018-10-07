import threading
class ThreadWorker():
    def __init__(self,func):
        self.thread = None
        self.data = None
        self.fun = func
        self.func = self.save_data(func)
        self.par = None

    def save_data(self,func):
        '''modify function to save its returned data'''
        def new_func(*args, **kwargs):
            self.data=func(self,*args, **kwargs)
        
        return new_func

    def start(self,params):
        self.data = None
        self.par = params
        if self.thread is not None:
            if self.thread.isAlive():
                return 'running' #could raise exception here

        #unless thread exists and is alive start or restart it
        
        self.thread = threading.Thread(target=self.func,args=params)
        self.thread.start()
        return 'started'

    def status(self):
        if self.thread is None:
            return 'not_started'
        else:
            if self.thread.isAlive():
                return 'running'
            else:
                return 'finished'
            
  
    def get_results(self):
        if self.thread is None:
            return 'not_started' #could return exception
        else:
            if self.thread.isAlive():
                return 'running'
            else:
               # cv2.imshow("image", self.data)
                out = self.data
                #if self.fun == camPreview:
                 #   self.thread = threading.Thread(target=self.func,args=self.par)
                  #  self.thread.start()
                return out
                
def add(x,y):
    return x +y
