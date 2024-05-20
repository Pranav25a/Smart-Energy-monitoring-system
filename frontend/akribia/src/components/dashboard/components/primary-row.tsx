import Dashbox from "./dashbox";
import InitialBox from "./initbox";

interface primaryrowProps {
  // Props, if needed, go here
}

const PrimaryRow: React.FC<primaryrowProps> = () => {

const PowerRow =(rowIdx:number)=>{
  const colorBox = [
    'text-red-500','text-yellow-500','text-blue-500'
  ]
  return(
    <div className="flex flex-col lg:w-1/3 gap-y-2">
    <Dashbox label={`voltage${rowIdx}`} value={`Line ${rowIdx}`} icon={`fluent:circle-off-16-filled`} iconClassName={colorBox[rowIdx-1]}/>
    <Dashbox label={`current${rowIdx}`} value={`Line ${rowIdx}`} icon={`fluent:circle-off-16-filled`} iconClassName={colorBox[rowIdx-1]} />
    <Dashbox label={`power${rowIdx}`} value={`Line ${rowIdx}`} icon={`fluent:circle-off-16-filled`} iconClassName={colorBox[rowIdx-1]}/>
    <Dashbox label={`frequency${rowIdx}`} value={`Line ${rowIdx}`} icon={`fluent:circle-off-16-filled`} iconClassName={colorBox[rowIdx-1]}/>
    </div>
  )
}

  return (
    <div>
    <div>
    <InitialBox/>
    </div>
    <div className="pt-4 flex flex-col lg:flex-row gap-x-2">   
     {/* Power */}
      {PowerRow(1)}
      {PowerRow(2)}
      {PowerRow(3)}

    </div>
    </div>
  );
};
export default PrimaryRow;
