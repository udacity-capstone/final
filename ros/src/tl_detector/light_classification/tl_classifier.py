from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np

class TLClassifier(object):
    def __init__(self, is_sim):

        self.label_map = dict()
        self.label_text = dict()
        if is_sim:
            PATH_TO_GRAPH = r'light_classification/model/simulator/frozen_inference_graph.pb'
            self.label_map = {1: TrafficLight.YELLOW, 2: TrafficLight.RED, 3: TrafficLight.GREEN}
            self.label_text = {1: "Yellow", 2: "Red", 3: "Green"}
        else:
            PATH_TO_GRAPH = r'light_classification/model/site/frozen_inference_graph.pb'
            red_classes = [4, 5, 7, 9, 13]
            green_classes = [2, 3, 6, 8, 11, 12]
            yellow_classes = [10]
            unknown_classes = [1]
            for i in red_classes:
                self.label_map[i] = TrafficLight.RED
                self.label_text[i] = "Red"
            for i in green_classes:
                self.label_map[i] = TrafficLight.GREEN
                self.label_text[i] = "Green"
            for i in yellow_classes:
                self.label_map[i] = TrafficLight.YELLOW
                self.label_text[i] = "Yellow"
            for i in unknown_classes:
                self.label_map[i] = TrafficLight.UNKNOWN
                self.label_text[i] = "Unknown"
        self.graph = tf.Graph()
        self.threshold = .5
        self.is_sim = is_sim
        with self.graph.as_default():
            graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_GRAPH, 'rb') as fid:
                graph_def.ParseFromString(fid.read())
                tf.import_graph_def(graph_def, name='')

            self.image_tensor = self.graph.get_tensor_by_name('image_tensor:0')
            self.boxes = self.graph.get_tensor_by_name('detection_boxes:0')
            self.scores = self.graph.get_tensor_by_name('detection_scores:0')
            self.classes = self.graph.get_tensor_by_name('detection_classes:0')
            self.num_detections = self.graph.get_tensor_by_name('num_detections:0')

        self.sess = tf.Session(graph=self.graph)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        with self.graph.as_default():
            img_expand = np.expand_dims(image, axis=0)
            (boxes, scores, classes, num_detections) = self.sess.run(
                [self.boxes, self.scores, self.classes, self.num_detections],
                feed_dict={self.image_tensor: img_expand})

        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)
        print('SCORES: ', scores[0])
        print('CLASSES: ', classes[0])

        if scores[0] > self.threshold:
            print(self.label_text[classes[0]])
            return self.label_map[classes[0]]
        return TrafficLight.UNKNOWN
