#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <iterator>
#include <string>
using namespace std;

#define SIGN_TO_UNSIGN_32_INCREMENT 0x80000000
#define ROW_WIDTH 8

int main(int argc, char *argv[]) {
	if (argc < 2) {
		cout << "Converter requires filename argument, aborting." << endl;
		return 1;
	}

	// Read from file into samples buffer
	ifstream raw_file(argv[1], ios::in | ios::binary);
	if (!raw_file.is_open()) {
		// Invalid file, did not open
		cout << "Invalid file - could not open, aborting." << endl;
		return 1;
	}

	unsigned int read_holder;
	vector<unsigned int> samples;
	while (raw_file.read(reinterpret_cast<char *>(&read_holder), sizeof(read_holder))) {
		samples.push_back(read_holder);
	}

	// "de-overflow" from signed 32-bit ints to unsigned 32-bit ints
	// also, downsample by 4x into resampled_buffer
	vector<unsigned int> resampled_buffer;
	for (auto it = samples.begin(); it != samples.end(); /* it inc done inside */) {
		resampled_buffer.push_back( *it + SIGN_TO_UNSIGN_32_INCREMENT );
		it += 4;
	}

	// Convert filename to header file
	string arg(argv[1]);
	auto filename_start = arg.find_last_of("/") + 1;
	string filename = arg.substr(filename_start, arg.find_last_of(".") - filename_start);
	filename = "wavetable_headers/" + filename + ".h";
	cout << "Converting: " << argv[1] << " -> " << filename << endl;

	// Prompt user for table name in header file
	string arr_name;
	cout << "Enter a name for this wavetable (must be a valid C variable name): ";
	cin >> arr_name;
	cout << "You entered: " << arr_name << " - generating header file..." << endl;

	// Derive include guard definition from array name
	string header_defname = arr_name;
	for (char &x : header_defname) x = toupper(x);

	/*
	 * WRITE RESAMPLED WAVETABLE TO FILE
	*/

	// Open file stream
	ofstream wtab_header(filename);

	// Include guard and array definition
	wtab_header << "#ifndef " << header_defname << "\n";
	wtab_header << "#define " << header_defname << "\n\n";
	wtab_header << "const unsigned int " << arr_name << "[] = {\n";

	// Save initial formatting for output stream
	ios init(NULL);
	init.copyfmt(wtab_header);

	// Prepare for array loop by setting hex configuration
	wtab_header << setfill('0') << hex;

	// Array loop
	int cols_in_row = 0;
	int samples_written = 0;
	for (unsigned int &i : resampled_buffer) {
		// Newline/indendation for rows
		if (cols_in_row == ROW_WIDTH) {
			cols_in_row = 0;
			wtab_header << "\n";
		}
		if (!cols_in_row) wtab_header << "    ";

		// Write value
		wtab_header << "0x" << setw(8) << i;

		cols_in_row++;
		samples_written++;
		// All list elements except last get a comma after them
		if (samples_written < resampled_buffer.size()) {
			wtab_header << ",";
			if (cols_in_row < ROW_WIDTH) wtab_header << " ";
		}
	}

	// Set configuration back to original config for remainder of header file
	wtab_header.copyfmt(init);

	// Array end, array length definition, end include guard
	wtab_header << "\n};\n";
	wtab_header << "unsigned int len_" << arr_name << " = " << resampled_buffer.size() << ";\n\n";
	wtab_header << "#endif";
}