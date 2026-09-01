import numpy as np
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import Ridge
import matplotlib.pyplot as plt
import joblib

def train_koopman():
    # 1. Load the dataset collected from ROS 2
    data = np.load('sam_koopman_dataset.npz')
    X = data['X']  # Shape: (N, 12)
    U = data['U']  # Shape: (N, 6)
    
    # Create shifted dataset pairs (current state -> next state)
    X_k = X[:-1]
    U_k = U[:-1]
    X_k_next = X[1:]
    
    # 2. Lift into Observable Space (EDMD)
    # Using degree=2 polynomials for nonlinear basis functions
    poly = PolynomialFeatures(degree=2, include_bias=False)
    
    print("Lifting states...")
    # Stack states and inputs to capture coupled dynamics
    XU_k = np.hstack((X_k, U_k)) 
    Z_k = poly.fit_transform(XU_k)
    
    # 3. Train Koopman Operator using Ridge Regression (Tikhonov Regularization)
    print(f"Training Koopman matrix with {Z_k.shape[1]} observable features...")
    operator = Ridge(alpha=0.1) 
    operator.fit(Z_k, X_k_next)
    joblib.dump(poly, 'koopman_basis.pkl')
    joblib.dump(operator, 'koopman_operator.pkl')
    print("Saved Koopman Python objects to disk.")
    
    # 4. Evaluate the Model
    score = operator.score(Z_k, X_k_next)
    print(f"Training R^2 Score: {score:.4f}")
    
    # 5. Predict one step ahead to verify
    X_pred = operator.predict(Z_k)
    
    # Plot predicted vs actual Surge velocity (u)
    plt.figure(figsize=(10, 4))
    plt.plot(X_k_next[:, 6], label='Actual Surge (Unity)', color='blue')
    plt.plot(X_pred[:, 6], label='Predicted Surge (Koopman)', color='red', linestyle='--')
    plt.title("Koopman System ID Validation")
    plt.legend()
    plt.show()

if __name__ == "__main__":
    train_koopman()