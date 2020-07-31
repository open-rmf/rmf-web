export type IconContextType = Record<string, Record<string, string>>
const axios = require('axios').default;
export default class IconManager {

    static getIconsConfigurationFile = async (): Promise<IconContextType> => {
        try {
            const response = await axios.get('/assets/icons/icons.json')
            return response.data as IconContextType;
        } catch (error) {
            console.error(error)
            return {}
        }
    }

    static getRobotIcon(icons: IconContextType, model: string): string | null {
        return icons.robots ? `/assets/icons${IconManager.getIcon(icons.robots, model)}` : null;
    }

    private static getIcon(icons: Record<string, string>, key: string): string | null {
        return icons.hasOwnProperty(key) ? icons[key] : null;
    }
}
