
class PriorityQueue:
    
    def  __init__(self):
        self.heap = []
        
    def push(self, item, priority):
        
        element = (item, priority)
            
        if self.isEmpty():
            self.heap.append(element)
        else:
            
            done = False
            for i in range(len(self.heap)):
                
                if self.heap[i][1] > priority:
                    self.heap.insert(i, element)
                    done = True
                    break
            
            if not done:
               self.heap.append(element) 
                    
    def pop(self):
        
        item, _ = self.heap.pop(0)
        return item

    def isEmpty(self):
        return len(self.heap) == 0

        
   
