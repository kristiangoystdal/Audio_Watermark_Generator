import pandas as pd
import os
import matplotlib.pyplot as plt

# Settings
FOLDER = "filtered_segments"
COLUMN_INDEX = 1  # Change this to plot a different column
FILE_INDEX = 1  # Change this to select a different file

# List CSV files in folder
csv_files = [f for f in os.listdir(FOLDER) if f.endswith(".csv")]
if not csv_files:
    raise FileNotFoundError(f"No CSV files found in '{FOLDER}'.")

csv_file = csv_files[FILE_INDEX]
file_path = os.path.join(FOLDER, csv_file)

# Load data
df = pd.read_csv(file_path)

# Plot selected column
filtered_values = df.iloc[:, COLUMN_INDEX][df.iloc[:, COLUMN_INDEX] > 1.3]
plt.plot(filtered_values.index, filtered_values.values)
plt.title(f"Plot of values > 1.3 in column {COLUMN_INDEX} of {csv_file}")
plt.xlabel("Row Index")
plt.ylabel(df.columns[COLUMN_INDEX])
plt.show()

# Calculate bits: 1 if value > 1.65 (high), else 0 (low), but exclude values > 2.5
column_values = df.iloc[:, COLUMN_INDEX]
valid_mask = column_values <= 2.5
bits = ((column_values > 1.65) & valid_mask).astype(int)

# Print bits or plot them
print("Bits (1=high, 0=low), excluding values > 2.5:")
print(bits.values)
print(len(bits))
