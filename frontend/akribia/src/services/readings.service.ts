import Api from "../api/instance"
import { Readings } from "../models/readings"

const baseURL = "readings/"
const getByDeviceId = (deviceID: number): Promise<Readings[]> => {
    return Api.get(`${baseURL}${deviceID}`) // Ensure proper URL formatting
        .then((response) => {
            if (Array.isArray(response)) {
                 // Ensure the response is an array
                return response.map(data => new Readings().deserialize(data));
            } else {
                throw new Error("Response is not an array");
            }
        })
        .catch((error) => {
            console.error("Error fetching readings by device ID:", error);
            throw error; // Re-throw the error to be handled by the caller
        });
};

const getByDeviceRange = (deviceID: number,date:any): Promise<Readings[]> => {
    return Api.get(`${baseURL}${deviceID}/range?startDate=${date?.[0]}&endDate=${date?.[1]}`) // Ensure proper URL formatting
        .then((response) => {
            if (Array.isArray(response)) {
                 // Ensure the response is an array
                return response.map(data => new Readings().deserialize(data));
            } else {
                throw new Error("Response is not an array");
            }
        })
        .catch((error) => {
            console.error("Error fetching readings by device ID:", error);
            throw error; // Re-throw the error to be handled by the caller
        });
};

const getCurrentData = (deviceID: number): Promise<Readings> => {
    return Api.get(`${baseURL}${deviceID}/currentData`) // Ensure proper URL formatting
        .then((response) => {
                return new Readings().deserialize(response);
        })
        .catch((error) => {
            console.error("Error fetching readings by device ID:", error);
            throw error; // Re-throw the error to be handled by the caller
        });
};



const ReadingService ={
getByDeviceId,
getByDeviceRange,
getCurrentData
}

export default ReadingService;