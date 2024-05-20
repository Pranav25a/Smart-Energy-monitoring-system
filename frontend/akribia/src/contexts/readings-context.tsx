import { createContext } from "react";
import { Readings } from "../models/readings";

export interface ReadingsContextType {
  readings: Readings[] | undefined;
  currentReadings: Readings | undefined;
  setCurrentReadings: React.Dispatch<React.SetStateAction<Readings | undefined>>;
  setReadings: React.Dispatch<React.SetStateAction<Readings[] | undefined>>;
  setEndDate: React.Dispatch<React.SetStateAction<string | undefined>>;
  setFromDate: React.Dispatch<React.SetStateAction<string | undefined>>;
  fromDate: string | undefined;
  endDate: string | undefined;
  setDeviceId: React.Dispatch<React.SetStateAction<number | undefined>>;
  deviceId: number | undefined;
  fetchDate: () => void;
  fetchCurrentData: () => void;
}


export const ReadingsContext = createContext<ReadingsContextType>({
  readings: undefined,
  currentReadings: undefined,
  setCurrentReadings: () => {},
  setReadings: () =>{},
  setEndDate: () => {},
  setFromDate: () => {},
  fromDate: undefined,
  endDate: undefined,
  setDeviceId: () => {},
  deviceId: undefined,
  fetchDate: () => {},
  fetchCurrentData: () => {}
});
