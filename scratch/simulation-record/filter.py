for path in ['GTHigh.tcl', 'GTLow.tcl', 'FreTexas.tcl', 'TimesSquareHigh.tcl']:
    with open(path, 'r') as file:
        lines = file.readlines()

    total_time = lines[-1].split()[2]

    # Filter the lines based on the conditions provided
    filtered_lines = []

    for line in lines:
        if 'set ' in line:
            filtered_lines.append(line)
        else:
            # Extract the time from the line and check if it is greater than 570
            time = float(line.split()[2])
            if 0.4 * float(total_time) < float(time) < 0.5 * float(total_time):
                filtered_lines.append(line)

    # Saving the filtered lines back to the original file
    filtered_file_path = 'filtered' + path
    with open(filtered_file_path, 'w') as file:
        file.writelines(filtered_lines)

