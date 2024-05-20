import React, { useEffect, useState } from 'react';

import Navbar from "../navbar";
import PrimaryRow from "./components/primary-row";
import { ReadingsContext } from '../../contexts/readings-context'; // Ensure this path matches your project structure
import { Readings } from '../../models/readings';
import ReadingService from '../../services/readings.service';

const Dashboard = () => {
  const [fromDate, setFromDate] = useState<string>();
  const [endDate, setEndDate] = useState<string>();
  const [readings, setReadings] = useState<Readings[]>();
  const [deviceId,setDeviceId] = useState<number>();
  const [currentReadings,setCurrentReadings] = useState<Readings>();

  const fetchDate = () => {
    if(fromDate && endDate && deviceId){
      ReadingService.getByDeviceRange(deviceId,[fromDate,endDate]).then((res)=>{
        setReadings(res);
      });
      
      
    }
    else{
      return;
    }
    
  }
  const fetchCurrentData = () => {
    if (deviceId) {
      ReadingService.getCurrentData(deviceId).then((res) => {
        setCurrentReadings(res);
      });
    }
  };

  useEffect(() => {
    const interval = setInterval(fetchCurrentData, 3000); 
    return () => clearInterval(interval);
  }, [deviceId]);

  return (
    <ReadingsContext.Provider value={{ readings, setReadings, fromDate, endDate, setFromDate, setEndDate,setDeviceId,deviceId, fetchDate,currentReadings,setCurrentReadings,fetchCurrentData }}>
      <div className="p-4">
        <Navbar/>
        <PrimaryRow />
      </div>
    </ReadingsContext.Provider>
  );
};

export default Dashboard;
function fetchCurrentData() {
  throw new Error('Function not implemented.');
}

