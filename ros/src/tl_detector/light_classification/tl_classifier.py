import cv2
import numpy as np
import tensorflow.contrib.keras as keras
from styx_msgs.msg import TrafficLight
import rospy

MODEL_WEIGHT_PATH = "/home/student/tl_model.h5"

def get_train_model():

    img_input = keras.layers.Input(shape=(224, 224, 3), name='vgg_input', dtype='float32')

    vgg16_model = keras.applications.VGG16(weights='imagenet', include_top=False, input_shape=(224, 224, 3))
    vgg16_model.trainable = False

    vgg16_output = vgg16_model(img_input)
    x = keras.layers.Flatten()(vgg16_output)
    x = keras.layers.Dense(1024, activation='elu')(x)
    preds = keras.layers.Dense(4, activation='softmax')(x)

    model = keras.models.Model(inputs=img_input, outputs=preds)
    adam = keras.optimizers.Adam(lr=0.0001)
    model.compile(optimizer=adam, loss="binary_crossentropy", metrics=['accuracy'])

    return model


class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        self.model = get_train_model()
        self.model.load_weights(MODEL_WEIGHT_PATH)
        self.model._make_predict_function()
        self.TL_dict = {0: TrafficLight.RED, 1: TrafficLight.YELLOW, 2: TrafficLight.GREEN, 3: TrafficLight.UNKNOWN}

        # for debug
        self.TL_dict2 = {0: "TrafficLight.RED", 1: "TrafficLight.YELLOW", 2: "TrafficLight.GREEN", 3: "TrafficLight.UNKNOWN"}

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction

        image = cv2.resize(image, (224, 224))
        pred = self.model.predict(np.array([image]))

        ans = self.TL_dict[np.argmax(pred)]
        ans2 = self.TL_dict2[np.argmax(pred)]

        # for debug
        rospy.loginfo("predict:{}, {}".format(ans, ans2))

        return self.model.predict(np.array([image]))

