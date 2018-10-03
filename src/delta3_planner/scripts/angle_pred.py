import keras
from keras.models import Sequential, Model
from keras.layers import Dense, Input
from keras.wrappers.scikit_learn import KerasRegressor
from sklearn.model_selection import cross_val_score
from sklearn.model_selection import KFold
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import Pipeline
from clean import *

def get_model():
    inputs = Input(shape=( 3,))
    x = Dense(6, activation='relu')(inputs)
    x = Dense(6, activation='relu')(x)
    x = Dense(6, activation='relu')(x)
    pred1 = Dense(1, activation='linear', name="arm1")(x)
    pred2 = Dense(1, activation='linear', name="arm2")(x)
    pred3 = Dense(1, activation='linear', name="arm3")(x)
    model = Model(inputs=inputs, outputs=[pred1, pred2, pred3])
    model.compile(optimizer='adam', loss='mse')
    return model

data = get_data()
model = get_model()
X = data[:, 7:10]
y = [data[:, 4], data[:, 5], data[:, 6]]
model.fit(X, y, epochs = 1000)



    

