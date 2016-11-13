ISR_FREQUENCY = 100000 # 100 kHz DDS sample frequency
INCREMENTS_TO_OVERFLOW = 268435456 # 2^28 = value at which next sequencer step begins

c1 = 32.7032 # frequency (in hertz) of C1
twelfth_root_of_two = 2 ** (1.0/12) # for semitone intervals
freqs_to_c5 = [*0..48].map {|x| (c1 * (twelfth_root_of_two ** x))} # frequencies from C1 to C5

tempo_vals = []
64.times {|x| tempo_vals << (70 + (2 * x))} # tempos from 70 to 196, in even increments

# print tempos
puts "Tempos:"
tempo_vals.each_with_index {|tempo| puts tempo}

# 4 quarter notes incremented per beat, so hz is frequency per quarter note
# tempo / 60 = beats per second -> multiply by 4 to get frequency per quarter note
tempo_freq_hz = tempo_vals.map {|t| t.to_f/15}

# freq = 1 / ( ( INCREMENTS_TO_OVERFLOW / PHASE_INCREMENT ) * ( 1 / ISR_FREQUENCY ) )
# 1 / freq = ( INCREMENTS_TO_OVERFLOW * ( 1 / ISR_FREQUENCY ) ) / PHASE_INCREMENT
# PHASE_INCREMENT = INCREMENTS_TO_OVERFLOW * ( 1 / ISR_FREQUENCY ) * freq
# round it to the nearest int because we store these as ints

phase_increments = tempo_freq_hz.map {|x| ( INCREMENTS_TO_OVERFLOW * (1.0/ISR_FREQUENCY) * x).round }

# print phase increments
puts "Phase accumulators:"
phase_increments.each_with_index {|acc| puts acc}