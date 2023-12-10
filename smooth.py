import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.interpolate import UnivariateSpline

def fit_spline(input_csv, output_csv, smoothness=0.0):
    # Read CSV file containing points
    df = pd.read_csv(input_csv, sep=',')
    
    # Extract x and y coordinates
    x = df['X'].values
    y = df['Y'].values

    # Create a spline interpolation
    t = np.arange(len(x))
    t_new = np.linspace(0, len(x) - 1, 1000)
    
    # Use UnivariateSpline to create a smooth spline without passing through the points
    spline_x = UnivariateSpline(t, x, s=smoothness)
    spline_y = UnivariateSpline(t, y, s=smoothness)

    # Evaluate the spline at new points
    x_new = spline_x(t_new)
    y_new = spline_y(t_new)

    # Create a DataFrame for the new points
    df_new = pd.DataFrame({'X': x_new, 'Y': y_new})

    # Write the new points to a CSV file
    df_new.to_csv(output_csv, index=False)

    # Plot the original points and the spline
    plt.scatter(x, y, label='Original Points', color='blue')
    plt.plot(x_new, y_new, label=f'Spline (Smoothness={smoothness})', color='red')
    plt.title('Smooth Spline Interpolation of Raceline Points')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.show()

if __name__ == "__main__":
    input_csv = 'output_points.csv'  # Replace with the path to your input CSV file
    output_csv = 'output_points.csv'  # Replace with the desired output CSV file name
    smoothness = 0.9  # Adjust the smoothness parameter (0 for less fitted, higher for more fitted)
    fit_spline(input_csv, output_csv, smoothness)
