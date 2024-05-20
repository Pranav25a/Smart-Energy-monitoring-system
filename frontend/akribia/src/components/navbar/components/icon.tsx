import { Icon } from "@iconify/react/dist/iconify.js";

interface IconProps {
  name: string;
  className?: string;
}

const Icons: React.FC<IconProps> = ({ name, className }) => {
  return (
      <Icon icon={name} className={className} />
  );
};

export default Icons;
