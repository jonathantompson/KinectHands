if(!window.SiteRecruit_Globals){var SiteRecruit_Config={},SiteRecruit_Globals={},SiteRecruit_Constants={};SiteRecruit_Globals.parseFlag=!1,SiteRecruit_Globals.empty=!1,SiteRecruit_Constants.browser={},SiteRecruit_Constants.browser.internetExplorer="Microsoft Internet Explorer",SiteRecruit_Constants.browser.mozilla="Netscape",SiteRecruit_Globals.browserName=navigator.appName,SiteRecruit_Globals.browserVersion=parseInt(navigator.appVersion),SiteRecruit_Globals.isInternetExplorer=!1,SiteRecruit_Globals.isMozilla=!1,SiteRecruit_Globals.browserName==SiteRecruit_Constants.browser.internetExplorer&&SiteRecruit_Globals.browserVersion>3&&(SiteRecruit_Globals.isInternetExplorer=!0),SiteRecruit_Globals.browserName==SiteRecruit_Constants.browser.mozilla&&SiteRecruit_Globals.browserVersion>4&&(SiteRecruit_Globals.isMozilla=!0),SiteRecruit_Constants.cookieLifetimeType={},SiteRecruit_Constants.cookieLifetimeType.duration=1,SiteRecruit_Constants.cookieLifetimeType.expireDate=2,SiteRecruit_Constants.invitationType={},SiteRecruit_Constants.invitationType.standard=0,SiteRecruit_Constants.invitationType.email=1,SiteRecruit_Constants.invitationType.domainDeparture=2,SiteRecruit_Constants.cookieType={},SiteRecruit_Constants.cookieType.alreadyAsked=1,SiteRecruit_Constants.cookieType.inProgress=2,SiteRecruit_Constants.horizontalAlignment={},SiteRecruit_Constants.horizontalAlignment.left=0,SiteRecruit_Constants.horizontalAlignment.middle=1,SiteRecruit_Constants.horizontalAlignment.right=2,SiteRecruit_Constants.verticalAlignment={},SiteRecruit_Constants.verticalAlignment.top=0,SiteRecruit_Constants.verticalAlignment.middle=1,SiteRecruit_Constants.verticalAlignment.bottom=2,SiteRecruit_Config.cookieName="msresearch",SiteRecruit_Config.cookieDomain=".microsoft.com",SiteRecruit_Config.cookiePath="/",SiteRecruit_Constants.cookieJoinChar=":",SiteRecruit_Config.cookieLifetimeType=1,SiteRecruit_Config.cookieDuration=90;function SiteRecruit_KeepAlive(){function n(){this.inProgressCookieExists()&&setInterval("SiteRecruit_Globals.keepAlive.checkCookie()",this.keepAlivePollDelay)}function t(){if(this.inProgressCookieExists()){var n=SiteRecruit_Constants.cookieJoinChar,t=SiteRecruit_Config.cookieName+"="+SiteRecruit_Constants.cookieType.inProgress+n+escape(document.location)+n+ +new Date+n+this.id+"path="+SiteRecruit_Config.cookiePath;SiteRecruit_Config.cookieDomain!=""&&(t+="domain="+SiteRecruit_Config.cookieDomain),document.cookie=t}}function i(){var n=SiteRecruit_Config.cookieName+"="+SiteRecruit_Constants.cookieType.inProgress;return document.cookie.indexOf(n)!=-1?!0:!1}this.keepAlivePollDelay=1e3,this.id=Math.random(),this.attemptStart=n,this.checkCookie=t,this.inProgressCookieExists=i}SiteRecruit_Globals.keepAlive=new SiteRecruit_KeepAlive,SiteRecruit_Globals.keepAlive.attemptStart();function SiteRecruit_PageConfigurationBroker(){function n(n){this.initializeMapping();var t=this.getConfigurationForPage(n);t!=null&&this.loadConfiguration(t)}function t(){var n=this.urls,t=this.pages,i=this.priorities;n[0]="",t[0]="/library/svy/SiteRecruit_PageConfiguration_2944mt.js",i[0]=0}function i(n){for(var f=0,i=-1,e,r,u,t=0;t<this.urls.length;t++)e=new RegExp(this.urls[t],"i"),n.toString().search(e)!=-1&&(r=this.priorities[t],r<f||(i=t,f=r));return u=null,i<0||(u=this.pages[i]),u}function r(n){document.write('<script language="JavaScript" src="'+n+'"><\/script>')}this.urls=[],this.pages=[],this.priorities=[],this.start=n,this.initializeMapping=t,this.getConfigurationForPage=i,this.loadConfiguration=r}SiteRecruit_Globals.isEnUs=!1,SiteRecruit_Globals.isFR=!1,SiteRecruit_Globals.isDE=!1,SiteRecruit_Globals.isJA=!1,document.cookie.toString().toLowerCase().indexOf("gsslang=")!=-1?(document.cookie.toString().toLowerCase().indexOf("gsslang=en-us")!=-1&&(SiteRecruit_Globals.isEnUs=!0),document.cookie.toString().toLowerCase().indexOf("gsslang=fr")!=-1&&(SiteRecruit_Globals.isFR=!0),document.cookie.toString().toLowerCase().indexOf("gsslang=de")!=-1&&(SiteRecruit_Globals.isDE=!0),document.cookie.toString().toLowerCase().indexOf("gsslang=ja")!=-1&&(SiteRecruit_Globals.isJA=!0)):((navigator.language&&navigator.language.toString().toLowerCase()=="en-us"||navigator.userLanguage&&navigator.userLanguage.toString().toLowerCase()=="en-us")&&(SiteRecruit_Globals.isEnUs=!0),(navigator.language&&navigator.language.toString().toLowerCase()=="fr"||navigator.userLanguage&&navigator.userLanguage.toString().toLowerCase()=="fr")&&(SiteRecruit_Globals.isFR=!0),(navigator.language&&navigator.language.toString().toLowerCase()=="de"||navigator.userLanguage&&navigator.userLanguage.toString().toLowerCase()=="de")&&(SiteRecruit_Globals.isDE=!0),(navigator.language&&navigator.language.toString().toLowerCase()=="ja"||navigator.userLanguage&&navigator.userLanguage.toString().toLowerCase()=="ja")&&(SiteRecruit_Globals.isJA=!0));try{(SiteRecruit_Globals.isInternetExplorer||SiteRecruit_Globals.isMozilla)&&(SiteRecruit_Globals.isEnUs||SiteRecruit_Globals.isFR||SiteRecruit_Globals.isDE||SiteRecruit_Globals.isJA)&&(SiteRecruit_Globals.broker=new SiteRecruit_PageConfigurationBroker,SiteRecruit_Globals.broker.start(window.location))}catch(e){}SiteRecruit_Globals.parseFlag=!0};