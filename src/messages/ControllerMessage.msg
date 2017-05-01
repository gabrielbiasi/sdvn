message ControllerMessage {
    
	int sourceVehicle;
	
	int messageType;
    int flowAction;
    int flowId;
	
	simtime_t timestamp;
	simtime_t lastUsed;
	
	simtime_t hardTimeout;
	simtime_t idleTimeout;
	
	int destinationAddress;
}
