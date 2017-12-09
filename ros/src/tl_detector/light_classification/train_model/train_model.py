import os
import cv2
import json
import numpy as np
import tensorflow.contrib.keras as keras
# from tensorflow.contrib.keras import backend as K

import generator


def load_json(path):

    with open(path) as file:

        json_file = json.load(file)

    return json_file


def get_one_hot_label(label):

    labels_dict = {"red":0, "yellow":1, "green":2, "unknown":3}

    one_hot = np.zeros(len(labels_dict))
    one_hot[labels_dict[label]] = 1

    return one_hot

def get_callbacks(model_path):

    os.makedirs(os.path.dirname(model_path), exist_ok=True)
    model_checkpoint = keras.callbacks.ModelCheckpoint(filepath=model_path, save_best_only=True, verbose=1)

    reduce_learning_rate_callback = keras.callbacks.ReduceLROnPlateau(factor=0.5, patience=2, verbose=1)
    early_stop_callback = keras.callbacks.EarlyStopping(patience=5, verbose=1)

    return [model_checkpoint, reduce_learning_rate_callback, early_stop_callback]


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


def main():

    WIDTH = 224
    HEIGHT = 224

    # path to dataset
    data_directory = "path/to/tl_images_dataset"

    # path to save model
    tl_model_path = os.path.join(data_directory, "tl_model.h5")

    # load json files representing image and label paths
    train_image_json_path = os.path.join(data_directory, "json", "train_data.json")
    train_label_json_path = os.path.join(data_directory, "json", "train_label.json")

    valid_image_json_path = os.path.join(data_directory, "json", "valid_data.json")
    valid_label_json_path = os.path.join(data_directory, "json", "valid_label.json")

    train_image_paths = load_json(train_image_json_path)
    valid_image_paths = load_json(valid_image_json_path)

    # get full path to each image
    train_image_paths = [os.path.join(data_directory, p) for p in train_image_paths]
    valid_image_paths = [os.path.join(data_directory, p) for p in valid_image_paths]

    train_labels = load_json(train_label_json_path)
    valid_labels = load_json(valid_label_json_path)

    batch_size = 4

    train_batch_generator = generator.BatchGenerator(train_image_paths, train_labels, batch_size, WIDTH, HEIGHT)
    valid_batch_generator = generator.BatchGenerator(valid_image_paths, valid_labels, batch_size, WIDTH, HEIGHT)

    # get model
    model = get_train_model()

    # train
    model.fit_generator(
        train_batch_generator.get_batch_generator(),
        steps_per_epoch=train_batch_generator.get_images_count() / batch_size,
        epochs=1,
        validation_data=valid_batch_generator.get_batch_generator(),
        validation_steps=valid_batch_generator.get_images_count() / batch_size,
        callbacks=get_callbacks(tl_model_path)
    )

    # test
    test_image_json_path = os.path.join(data_directory, "json", "test_data.json")
    test_label_json_path = os.path.join(data_directory, "json", "test_label.json")

    test_image_paths = load_json(test_image_json_path)
    test_image_paths = [os.path.join(data_directory, p) for p in test_image_paths]
    test_labels = load_json(test_label_json_path)

    test_batch_generator = generator.BatchGenerator(test_image_paths, test_labels, batch_size, WIDTH, HEIGHT)

    loss, accuracy = model.evaluate_generator(
        test_batch_generator.get_batch_generator(),
        test_batch_generator.get_images_count() / batch_size
    )

    print("Test Accuracy = {:.2f}".format(accuracy))


if __name__ == "__main__":
    main()
