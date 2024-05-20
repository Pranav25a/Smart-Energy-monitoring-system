
import { useContext, useState } from "react";
import DateComponent from "./components/date";
import Icon from "./components/icon";
import Title from "./components/title";
import { ReadingsContext } from "../../contexts/readings-context";
import DropdownCustom from "../dashboard/components/dropdown";

interface navbarProps {
 
}
const Navbar:React.FC<navbarProps> = () =>{
  const {fetchDate} = useContext(ReadingsContext);
  const [isFetching, setIsFetching] = useState(false);

  const handleFetchClick = () => {
    setIsFetching(true); // Indicate that fetching has started
    fetchDate(); // Call the fetchDate function from the context

    // Optionally set a timeout to reset the fetching state
    setTimeout(() => {
      setIsFetching(false);
    }, 1000); // Reset after 1 second (adjust as needed)
  };
  const refreshIconClasses = `text-white text-xl cursor-pointer ${isFetching ? 'fetching' : ''}`;

  return (
    <div className="flex justify-between shadow-lg rounded-md p-2 shadow-gray-500 bg-gray-400 ">
      <Title icon="simple-icons:googleanalytics" iconClassName="text-2xl" value="Coolp Power Analytics" valueClassName="text-white font-sans lg:text-lg text-xs font-light pl-2 "/>
      <div className="flex gap-2 items-center">
        <DropdownCustom/>
        <DateComponent />
        <div onClick={handleFetchClick}>
        <Icon name={"solar:refresh-bold"} className={refreshIconClasses}/>
        </div>
      </div>
    </div>
  );
}
export default Navbar;