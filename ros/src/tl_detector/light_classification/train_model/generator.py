import cv2
import sklearn.utils
import numpy as np


class BatchGenerator:

    def __init__(self, image_paths, labels, batch_size, width, height):

        self.image_paths = image_paths
        self.labels = labels
        self.batch_size = batch_size
        self.width = 224
        self.height = 224

        self.image_batch = []
        self.label_batch = []

        self.labels_dict = {"red":0, "yellow":1, "green":2, "unknown":3}

    def get_one_hot_label(self, label):

        one_hot = np.zeros(len(self.labels_dict))
        one_hot[self.labels_dict[label]] = 1

        return one_hot

    def get_images_count(self):

        return len(self.image_paths)

    def get_labels_count(self):

        return len(self.labels)

    def get_batch_generator(self):

        while True:

            sklearn.utils.shuffle(self.image_paths, self.labels)

            for image_path, label in zip(self.image_paths, self.labels):

                image = cv2.imread(image_path)
                one_hot_label = self.get_one_hot_label(label)

                if not(image.shape[0] == self.height and image.shape[1] == self.width):

                    image = cv2.resize(image, (self.height, self.width))

                self.image_batch.append(image)
                self.label_batch.append(one_hot_label)

                if (len(self.image_batch) is self.batch_size) and (len(self.label_batch) is self.batch_size):

                    yield (sklearn.utils.shuffle(np.array(self.image_batch), np.array(self.label_batch)))

                    self.image_batch.clear()
                    self.label_batch.clear()




