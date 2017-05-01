message NeighborMessage {
	int sourceVehicle;
	simtime_t timestamp;
    int neighbors[];
}
