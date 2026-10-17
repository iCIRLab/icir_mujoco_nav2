# Periodic local heartbeat. Receivers measure freshness with a steady clock.
# epoch changes on goal changes, stops and explicit resumes.
uint64 epoch
bool permitted
bool active
float64 linear
float64 angular
