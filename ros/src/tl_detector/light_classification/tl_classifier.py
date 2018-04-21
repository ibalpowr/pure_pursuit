import numpy as np
import cv2
import tensorflow as tf
import os

cwd = os.path.dirname(os.path.realpath(__file__))

class TLClassifier(object):
    def __init__(self):

        os.chdir(cwd)

        # setup tensorflow graph       
        CKPT = 'ssd-tf.pb'
        self.detection_graph = tf.Graph()
        
        # in case of cpu ... comment two lines below
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
       
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(CKPT, 'rb') as fid:
               serialized_graph = fid.read()
               od_graph_def.ParseFromString(serialized_graph)
               tf.import_graph_def(od_graph_def, name='')

            # in case of cpu ... comment the line below
            self.sess = tf.Session(graph=self.detection_graph, config=config)
            # in case of cpu ... uncomment the line below
            #self.sess = tf.Session(graph=self.detection_graph)
            
            self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')

            self.boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
            self.scores =self.detection_graph.get_tensor_by_name('detection_scores:0')
            self.classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
            self.num_detections =self.detection_graph.get_tensor_by_name('num_detections:0')

    def load_image(self, image):
         return np.asarray(image, dtype="uint8" )

    def detect_frame(self, image):  
        with self.detection_graph.as_default():
              image_expanded = np.expand_dims(image, axis=0)
              (boxes, scores, classes, num_detections) = self.sess.run(
                  [self.boxes, self.scores, self.classes, self.num_detections],
                  feed_dict={self.image_tensor: image_expanded})
             
              boxes=np.squeeze(boxes)
              classes =np.squeeze(classes)
              scores = np.squeeze(scores) 
    
              cls = classes.tolist()
             
              # highest score
              idx = 0;
              conf = scores[idx]
              cls_idx = cls[idx]
              
              # if low confidence   
              if scores[idx]<=0.9:
                  box=[0, 0, 0, 0]
                  print('low confidence ', scores[idx])
                  cls_idx = 4.0
              # a good detection   
              else:
                  box = boxes[idx]
                  print('confidence ', scores[idx])

        return box, conf, cls_idx


