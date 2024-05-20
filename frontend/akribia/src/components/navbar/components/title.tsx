import Icon from "./icon";

interface titleProps{
    value:string;
    icon?:string;
    iconClassName?:string;
    valueClassName?:string;
    boxClassName?:string;
}
const Title:React.FC<titleProps> = ({value,valueClassName,icon ="",iconClassName,boxClassName}) =>{
    return(
        <div className={`${boxClassName}`}>
            <div className={`flex gap-1 items-center pl-2`}>
            <Icon name={icon} className={iconClassName}/>
           <div className={valueClassName}>{value}</div>
           </div>
        </div>
    )
}

export default Title;