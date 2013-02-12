class Roles:
    #Words that need to be annotated with roles
    roleWord = ["V","N","AV","AD"]
    rolePath = "roles/"
    
    def __init__(self):
        self.roles = self.loadRoles()
        
        #Load the roles for all different word types
    def loadRoles(self):
        localRoles = [self.loadRolesForWordType(x) for x in self.roleWord]
        return localRoles
    
    #Load the roles for a specific word type
    def loadRolesForWordType(self,wordType):
        if wordType == "V":
            fileName = "verbRoles.db"
        elif wordType == "N":
            fileName = "nounRoles.db"
        elif wordType == "AD":
            fileName = "adjectiveRoles.db"
        elif wordType == "AV":
            fileName = "adverbRoles.db"
        f = open(self.rolePath + fileName, 'r')
        localRoles = self.parseRoleFile(f)
        f.close()
        return localRoles
        
    def parseRoleFile(self, f):
        localRoles = [line for line in f]
        return localRoles