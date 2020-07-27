export type IconContextType = Record<string, Record<string, string>>

export default class IconManager {

    static getIconsConfigurationFile = async (): Promise<IconContextType> => {
        const response: any = await fetch('https://github.com/matiasbavera/romi-dashboard-icons/blob/master/icons.json');
        console.log(response)
        return response.json() as IconContextType;
        // try {
        //     console.log(response)
        //     return <IconContextType>response.json();
        // } catch (error) {
        //     console.error(error)
        // }
    }

    static getRobotIcon(icons: IconContextType, model: string): string | null {
        return icons.robot ? IconManager.getIcon(icons.robot, model) : null;
    }

    private static getIcon(icons: Record<string, string>, key: string): string | null {
        return icons.hasOwnProperty(key) ? icons[key] : null;
    }
}
