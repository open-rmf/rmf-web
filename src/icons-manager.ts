export type IconContextType = Record<string, Record<string, string>>
const axios = require('axios').default;
export default class IconManager {

    static getIconsConfigurationFile = async (): Promise<IconContextType> => {
        const response = await axios.get('https://raw.githubusercontent.com/matiasbavera/romi-dashboard-icons/master/icons.json', { mode: 'no-cors' })
        return response.data as IconContextType;
        // try {
        //     console.log(response)
        //     return <IconContextType>response.json();
        // } catch (error) {
        //     console.error(error)
        // }
    }

    static getRobotIcon(icons: IconContextType, model: string): string | null {
        return icons.robots ? `/assets/icons${IconManager.getIcon(icons.robots, model)}` : null;
    }

    private static getIcon(icons: Record<string, string>, key: string): string | null {
        return icons.hasOwnProperty(key) ? icons[key] : null;
    }
}
