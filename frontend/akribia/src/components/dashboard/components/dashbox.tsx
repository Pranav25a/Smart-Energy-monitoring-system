import React from 'react';
import Title from "../../navbar/components/title";
import AreaChart from "./samplechart";

const Dashbox = ({ label,icon='',iconClassName='',value='',chartDiv=''}) => {
    return(
        <div className="flex flex-col shadow-lg rounded-md p-2 w-full bg-slate-100 shadow-slate-300 border border-t-1 border-slate-300">
            <Title value={value} icon={icon} iconClassName={iconClassName}/>
            <div className={`flex justify-center ${chartDiv}`}>
                <AreaChart label={label}/>
            </div>
        </div>
    );
};

export default Dashbox;
