import os

from tensorflow.python.keras.models import Sequential
from tensorflow.python.keras.layers import Conv2D, MaxPooling2D, Flatten, Dense


def loadModel() -> Sequential:
    model = Sequential([
        # Conv layers
        Conv2D(16, (3, 3), activation='relu', input_shape=(250, 250, 3)),
        MaxPooling2D(2, 2),
        Conv2D(16, (3, 3), activation='relu'),
        MaxPooling2D(2, 2),
        Conv2D(32, (5, 5), activation='relu'),
        MaxPooling2D(2, 2),
        Conv2D(32, (5, 5), activation='relu'),
        MaxPooling2D(2, 2),
        # Flatten layer
        Flatten(),
        # Fully connected layers
        Dense(128, activation='relu'),
        Dense(64, activation='relu'),
        Dense(32, activation='relu'),
        # Output neuron
        Dense(1, activation='sigmoid'),
        # TODO Dense(1, activation='linear') or scale parameter
    ])
    model.load_weights(os.path.join(os.path.dirname(__file__), './checkpoints/last_training'))
    return model