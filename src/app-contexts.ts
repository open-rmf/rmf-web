import { createContext } from "react";
import { IconContextType } from "./icons-manager";

/* Declares the IconContext which contains the icons used on the app*/
export const IconContext = createContext<IconContextType>({});