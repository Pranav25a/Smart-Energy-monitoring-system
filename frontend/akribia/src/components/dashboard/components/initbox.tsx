import React, { useContext, useState } from "react";
import Title from "../../navbar/components/title";
import { Tooltip } from "antd";
import { ReadingsContext } from "../../../contexts/readings-context";
import Icons from "../../navbar/components/icon";
import Dashbox from "./dashbox";

const Powerbox = (rowIdx?: number, isLoad?: boolean) => {
  const { currentReadings } = useContext(ReadingsContext);
  const getIndex = (value: string) => `${value}${rowIdx}`;
  const shouldPulse = !currentReadings;

  const displayValue = (key: string, prefix: string) =>
    currentReadings
      ? `${prefix}${rowIdx} : ${currentReadings[getIndex(key)]}`
      : "N/A";

  const tooltipTitle = "Please select a device";

  return (
    <div
      className={`flex ${
        isLoad ? "flex-row gap-x-1 justify-between" : "flex-col w-1/3 gap-y-2"
      }`}
    >
      <Tooltip title={shouldPulse ? tooltipTitle : ""}>
        <div
          className={`${shouldPulse ? "animate-pulse " : ""}${
            isLoad ? "w-1/3" : ""
          }`}
        >
          <Title
            value={`${isLoad ? "Line 1" : displayValue("voltage", "Voltage")}`}
            valueClassName={`${
              isLoad ? "font-medium text-sm" : "font-normal text-sm"
            }`}
            boxClassName={`${
              isLoad
                ? "flex justify-center border border-[0.5px] border-red-500 rounded hover:bg-red-100 h-[2.5rem]"
                : `flex p-0.5 border border-[0.5px] border-slate-500 rounded hover:bg-slate-200 h-[2rem]`
            }`}
            icon={`${
              isLoad
                ? "fluent:circle-off-16-filled"
                : "fluent-emoji-high-contrast:high-voltage"
            }`}
            iconClassName={`${isLoad ? "text-red-500" : ""}`}
          />
        </div>
      </Tooltip>

      <Tooltip title={shouldPulse ? tooltipTitle : ""}>
        <div
          className={`${shouldPulse ? "animate-pulse " : ""}${
            isLoad ? "w-1/3" : ""
          }`}
        >
          <Title
            value={`${isLoad ? "Line 2" : displayValue("current", "Current")}`}
            valueClassName={`${
              isLoad ? "font-medium text-sm" : "font-normal text-sm"
            }`}
            boxClassName={`${
              isLoad
                ? "flex justify-center border border-[0.5px] border-yellow-500 rounded hover:bg-yellow-100 h-[2.5rem]"
                : "flex p-0.5 border border-[0.5px] border-slate-500 rounded hover:bg-slate-200 h-[2rem]"
            }`}
            icon={`${
              isLoad ? "fluent:circle-off-16-filled" : "mdi:current-ac"
            }`}
            iconClassName={`${isLoad ? "text-yellow-500" : ""}`}
          />
        </div>
      </Tooltip>

      <Tooltip title={shouldPulse ? tooltipTitle : ""}>
        <div
          className={`${shouldPulse ? "animate-pulse " : ""}${
            isLoad ? "w-1/3" : ""
          }`}
        >
          <Title
            value={`${isLoad ? "Line 3" : displayValue("power", "Power")}`}
            valueClassName={`${
              isLoad ? "font-medium text-sm" : "font-normal text-sm"
            }`}
            boxClassName={`${
              isLoad
                ? "flex justify-center border border-[0.5px] border-blue-500 rounded hover:bg-blue-100 h-[2.5rem]"
                : "flex p-0.5 border border-[0.5px] border-slate-500 rounded hover:bg-slate-200 h-[2rem]"
            }`}
            icon={`${isLoad ? "fluent:circle-off-16-filled" : "ph:power-fill"}`}
            iconClassName={`${isLoad ? "text-blue-500" : ""}`}
          />
        </div>
      </Tooltip>

      {!isLoad && (
        <Tooltip title={shouldPulse ? tooltipTitle : ""}>
          <div className={`${shouldPulse ? "animate-pulse" : ""}`}>
            <Title
              value={displayValue("frequency", "Frequency")}
              valueClassName={"font-normal text-sm"}
              boxClassName={
                "flex p-0.5 border border-[0.5px] border-slate-500 rounded hover:bg-slate-200 h-[2rem]"
              }
              icon={"game-icons:lightning-frequency"}
              iconClassName={""}
            />
          </div>
        </Tooltip>
      )}
    </div>
  );
};

const InitialBox = () => {
  const [isFetching, setIsFetching] = useState(false);
  const { fetchCurrentData,currentReadings } = useContext(ReadingsContext);
  const handleFetchClick = () => {
    setIsFetching(true); // Indicate that fetching has started
    fetchCurrentData();

    setTimeout(() => {
      setIsFetching(false);
    }, 1000); // Reset after 1 second (adjust as needed)
  };

  const refreshIconClasses = `text-indigo-500 text-xl cursor-pointer ${
    isFetching ? "fetching" : ""
  }`;

  let createdAt = new Date(currentReadings?.createdAt).toLocaleDateString() +" "+new Date(currentReadings?.createdAt).toLocaleTimeString();
  return (
    <div className="pt-4 pl-0.5 flex gap-2 lg:flex-row flex-col w-full">
      <div className="flex flex-col lg:w-1/2 gap-y-4 p-3 bg-slate-100 shadow-slate-300 border border-t-1 border-slate-300 rounded shadow-md">
        <div className="pb-3 flex justify-between">
          <div className="flex flex-row gap-1 items-center">
            <span className="text-2xl font-light text-slate-500">
              Updated At :
            </span>
            <span className="text-md pt-1 text-slate-400">
              {createdAt}
            </span>
            <div onClick={handleFetchClick}>
              <Icons
                name={"solar:refresh-bold"}
                className={refreshIconClasses}
              />
            </div>
          </div>
          <div>
            <span className="text-slate-700 text-sm font-medium ">Sequence : </span>
            {currentReadings ?
            (currentReadings?.["sequence"]) ?
                <span className="text-xs p-1 border border-[0.5] border-green-600 bg-green-50 text-green-600 rounded">Pass</span>
                :
                <span className="text-xs p-1 border border-[0.5] border-red-600 bg-red-50 text-red-600 rounded">Fail</span>
                :
                <Tooltip title={"Select Device"}>
                <span className="text-xs p-1 border border-[0.5] border-slate-600 bg-slate-50 text-slate-600 rounded animate-pulse">Select Device</span>
                </Tooltip>
            }
          </div>
        </div>
        {Powerbox(0, true)}
        <div className="flex flex-row gap-x-1">
          {Powerbox(1)}
          {Powerbox(2)}
          {Powerbox(3)}
        </div>
      </div>
      <div className="lg:w-1/2">
        <Dashbox label={"power1"} chartDiv="h-[300px]"/>
      </div>
    </div>
  );
};

export default InitialBox;