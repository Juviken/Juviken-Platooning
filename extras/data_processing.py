"""
Data processing functions for the project.
"""

# Import statements
import pandas as pd
import os


# Function to load the data
def load_data(file_path):
    """
    Load the data from the file path. Expects a CSV file.

    Args:
        file_path (str): Path to the file.

    Returns:
        pandas.DataFrame: The loaded data.
    """
    try:
        data = pd.read_csv(file_path)
        return data
    except FileNotFoundError:
        print(f"Error: The file {file_path} was not found.")
        return None
    except pd.errors.EmptyDataError:
        print(f"Error: The file {file_path} is empty.")
        return None
    except pd.errors.ParserError:
        print(f"Error: The file {file_path} could not be parsed as a CSV.")
        return None


def main():
    directory = os.path.abspath("extras/sensor_data/")  # Ensure absolute path
    data_files = []

    print(f"Current working directory: {os.getcwd()}")

    # Collect files from the directory
    for root, dirs, files in os.walk(directory):
        for file in files:
            data_files.append(os.path.join(root, file))

    # Handle empty directory case
    if not data_files:
        print("No files found in the specified directory.")
        return

    # Print out data files as a list of options and wait for user input
    print("Select a file to load:")
    for i, file in enumerate(data_files):
        print(f"{i}: {file}")

    # Validate user input
    while True:
        try:
            file_index = int(input("Enter the index of the file you want to load: "))
            if 0 <= file_index < len(data_files):
                break
            else:
                print("Error: Index out of range. Try again.")
        except ValueError:
            print("Error: Invalid input. Please enter a valid integer.")

    # Load the data
    selected_file = data_files[file_index]
    print(f"Selected file: {selected_file}\n")
    print(f"Loading file: {selected_file}")
    data = load_data(selected_file)

    # Handle the case where data loading fails
    if data is not None:
        print("Data loaded successfully!")
        print(data.head())
    else:
        print("Failed to load the data. Please check the file.")


if __name__ == "__main__":
    main()
