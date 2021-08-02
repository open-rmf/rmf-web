import React from 'react';
export interface LoginCardProps {
  title: string;
  logo: string;
  children?: React.ReactNode;
}
export declare const LoginCard: React.ForwardRefExoticComponent<
  LoginCardProps & React.RefAttributes<HTMLDivElement>
>;
export default LoginCard;
