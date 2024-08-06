#!/usr/bin/env python3

import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import accuracy_score, confusion_matrix, classification_report
from joblib import dump

# Load your EEG data
data = pd.read_csv('frequency_dataset.csv')

# Inspect the data (first few rows)
print(data.head())

# Define the feature columns and the target column
features = ['alpha_0', 'alpha_1', 'alpha_2', 'alpha_3', 'alpha_4', 'alpha_5', 'alpha_6', 'alpha_7', 'alpha_8', 'alpha_9', 'alpha_10', 'alpha_11', 'alpha_12', 'alpha_13', 
            'beta_low_0', 'beta_low_1', 'beta_low_2', 'beta_low_3', 'beta_low_4', 'beta_low_5', 'beta_low_6', 'beta_low_7', 'beta_low_8', 'beta_low_9', 'beta_low_10', 'beta_low_11', 'beta_low_12', 'beta_low_13', 
            'beta_high_0', 'beta_high_1', 'beta_high_2', 'beta_high_3', 'beta_high_4', 'beta_high_5', 'beta_high_6', 'beta_high_7', 'beta_high_8', 'beta_high_9', 'beta_high_10', 'beta_high_11', 'beta_high_12', 'beta_high_13', 
            'gamma_0', 'gamma_1', 'gamma_2', 'gamma_3', 'gamma_4', 'gamma_5', 'gamma_6', 'gamma_7', 'gamma_8', 'gamma_9', 'gamma_10', 'gamma_11', 'gamma_12', 'gamma_13', 
            'theta_0', 'theta_1', 'theta_2', 'theta_3', 'theta_4', 'theta_5', 'theta_6', 'theta_7', 'theta_8', 'theta_9', 'theta_10', 'theta_11', 'theta_12', 'theta_13']

# The target column that contains the labels 'neutral', 'open', and 'close'
target = 'command'

# Extract features and target
X = data[features]
y = data[target]

# Split the data into training and testing sets
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.3, random_state=42)

# Standardize the features
scaler = StandardScaler()
X_train = scaler.fit_transform(X_train)
X_test = scaler.transform(X_test)

# Train a RandomForestClassifier
model = LogisticRegression(max_iter=1000, random_state=42)
model.fit(X_train, y_train)

# Predict on the test set
y_pred = model.predict(X_test)

# Evaluate the model
accuracy = accuracy_score(y_test, y_pred)
conf_matrix = confusion_matrix(y_test, y_pred)
class_report = classification_report(y_test, y_pred, target_names=['neutral', 'open', 'close'])

print(f'Accuracy: {accuracy}')
print('Confusion Matrix:')
print(conf_matrix)
print('Classification Report:')
print(class_report)

# Save the trained model and the scaler
dump(model, 'trained_model_logistic.joblib')
dump(scaler, 'scaler_logistic.joblib')
