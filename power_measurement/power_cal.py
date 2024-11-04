import csv

# Define the time interval between each reading in hours
# For example, if readings were taken every second, interval_in_hours = 1 / 3600
interval_in_seconds = 1
interval_in_hours = interval_in_seconds / 3600

# Initialize total current in Ampere-hours
total_current_ah = 0

# Read the CSV file
with open("without_log5.csv", "r") as csvfile:
    csvreader = csv.reader(csvfile)
    next(csvreader)  # Skip the header row

    # Loop through each row and add the current contribution
    for row in csvreader:
        current_amps = float(row[0])  # Convert current reading to a float
        # Calculate current contribution for this interval in Ampere-hours
        total_current_ah += current_amps * interval_in_hours

# Convert total current to milliampere-hours (mAh)
total_current_mAh = total_current_ah * 1000

print(f"Total Power Consumption: {total_current_mAh:.2f} mAh")
