window.Mst===undefined&&(window.Mst={}),Mst.FlyoutNavigationMenu=function(n,t,i,r,u,f,e,o,s,h,c,l,a){var p=null,y=".",v=this;v._menu=$("#"+n),v._menuClass=y+t,v._itemClass=y+i,v._flyoutLinkClass=y+r,v._staticLinkClass=y+u,v._flyoutClass=y+f,v._flyoutRegionClass=y+e,v._menuSectionClass=y+o,v._hideTimerId=p,v._showTimerId=p,v._preventClickTimerId=p,v._preventDwellTimerId=p,v._preventClick=!1,v._preventDwell=!1,v._Issubmenu=c,v._subflyoutClass=y+l,v._subitemClass=y+a,v._onOpenClick=s,v._onCloseEvent=h,$(document).click($.proxy(v.DocClick,v)),$(v._itemClass+" "+v._flyoutLinkClass,v._menu).click($.proxy(v.ItemClick,v)).mouseover($.proxy(v.ItemMouseover,v)),$(v._itemClass+" "+v._staticLinkClass,v._menu).mouseover($.proxy(v.HideFlyouts,v)),$(v._flyoutClass,v._menu).mouseover($.proxy(v.KeepFlyoutVisible,v))},Mst.FlyoutNavigationMenu.prototype={DocClick:function(n){var i=this._menu.find(".selected"),t;i.size()>0&&(t=$.contains(i.get(0),n.target),t||this.HideFlyouts())},ItemClick:function(n){var t=this;n.preventDefault(),t.ClearHideTimer(),t.ClearShowTimer(),t.ClearPreventDwellTimer(),t._preventDwell=!0,t._preventClick==!1&&(t.StartPreventDwellTimer(),t.IsFlyoutVisible(n)?(t.HideFlyouts(),t.HideSubMenuFlyouts(n)):(t.HideFlyouts(),t.ShowFlyout(n),t._onOpenClick!=null&&t._onOpenClick(n)))},ItemDwell:function(n){var t=this;t.ClearPreventClickTimer(),t._preventClick=!0,t._preventDwell==!1&&(t.StartPreventClickTimer(),t.IsFlyoutVisible(n)?t.HideFlyouts():(t.HideFlyouts(),t.ShowFlyout(n),t._onOpenDwell!=null&&t._onOpenDwell(n)))},IsFlyoutVisible:function(n){return $(n.target).parents(this._itemClass).find(this._flyoutClass).css("display")!="none"},ShowFlyout:function(n){var t=$(n.target).parents(this._itemClass).addClass("selected").find(this._flyoutClass);this.PositionFlyout(t),jQuery.browser.opera||jQuery.browser.msie&&document.documentMode<=7?t.show():t.slideDown(200)},HideFlyouts:function(n){var t=this;t.HideSubMenuFlyouts(n),jQuery.browser.opera||jQuery.browser.msie&&document.documentMode<=7?$(t._flyoutClass,t._menu).hide():$(t._flyoutClass,t._menu).slideUp(200);var i=t;t._onCloseEvent!=null&&$(t._itemClass).filter(".selected").each(function(){i._onCloseEvent($(this).find(i._flyoutLinkClass))}),$(t._itemClass).removeClass("selected")},HideSubMenuFlyouts:function(){var t=this;t._Issubmenu==!1&&(jQuery.browser.opera||jQuery.browser.msie&&document.documentMode<=7?$(t._subflyoutClass).hide():$(t._subflyoutClass).slideUp(200),$(t._subitemClass).removeClass("selected"))},KeepFlyoutVisible:function(n){n.stopPropagation(),this.ClearHideTimer(),this.ClearShowTimer()},PositionFlyout:function(n){var f=this,o="left",u=n.parents(f._itemClass).width(),s=n.parents(f._itemClass).position().left,e=n.outerWidth(),l=n.parents(f._menuClass).width(),a=n.parents(f._menuClass).position().left,c=l,i,r,h,t;n.parents(f._flyoutRegionClass).size()>0&&(c=n.parents(f._flyoutRegionClass).innerWidth()),i=s+a,r=l-s-u-5,document.documentElement.dir=="rtl"&&(o="right",u+=7,r=c-s-u,h=i,i=r,r=h),r<0&&(r=0),i<0&&(i=0),t=0,e>u+r?e<u+i?(t=document.documentElement.dir=="rtl"?-e+u+2:-e+u,n.css(o,t),n.addClass("dock-right")):e<i+r+u?(t=-e+(r+u),n.css(o,t)):(t=document.documentElement.dir=="rtl"?-i-1:-i-6,n.css(o,t)):(n.css(o,t),n.addClass("dock-left"))},StartShowTimer:function(n){this._showTimerId=window.setTimeout($.proxy(function(){this.ItemDwell(n)},this),100)},StartHideTimer:function(){this._hideTimerId=window.setTimeout($.proxy(this.HideFlyouts,this),1e3)},StartPreventClickTimer:function(){this._preventClickTimerId=window.setTimeout($.proxy(function(){this._preventClick=!1},this),350)},StartPreventDwellTimer:function(){this._preventDwellTimerId=window.setTimeout($.proxy(function(){this._preventDwell=!1},this),350)},ClearShowTimer:function(){window.clearTimeout(this._showTimerId),this._showTimerId=null},ClearHideTimer:function(){window.clearTimeout(this._hideTimerId),this._hideTimerId=null},ClearPreventClickTimer:function(){window.clearTimeout(this._preventClickTimerId),this._preventClickTimerId=null},ClearPreventDwellTimer:function(){window.clearTimeout(this._preventDwellTimerId),this._preventDwellTimerId=null}},Mst.FlyoutNavigationMenuV3=function(n,t,i){var u=null,r=this;r.Control=$("#"+n+"_mstNavMenu"),r.StageWidth=t,r.StagePadding=i,r._hideTimerId=u,r._showTimerId=u,r._preventClickTimerId=u,r._preventDwellTimerId=u,r._preventClick=!1,r._preventDwell=!1,$($.proxy(r.Ready,r))},Mst.FlyoutNavigationMenuV3.prototype={Ready:function(){var n=this;$(document).click($.proxy(n.DoClick,n)),n.SetFlyoutBackground(),$(window).resize($.proxy(n.SetFlyoutBackground,n)),$(".mstNavNavItem .mstNavNavItemTabText").click($.proxy(n.ItemClick,n))},DoClick:function(n){var i=this.Control.find(".selected"),t;i.size()>0&&(t=$.contains(i.get(0),n.target),t||(this.HideFlyouts(),this.BiTrack(n,10)))},SetFlyoutBackground:function(){var e="padding-right",u="padding-left",i=this,t=i.Control.offset().left,f=0,o=parseInt(i.StageWidth==""?$(".stage").width():i.StageWidth),s=parseInt(i.StagePadding==""?$(".stage").css(u):i.StagePadding),n=$(".mstNavNavItemFlyout"),r;n.css({left:"0px",right:"0px",width:"0px"}),document.documentElement.dir=="ltr"?(n.css("left",t*-1),n.css(u,t),n.css(e,t),f=o):(r=t<s?s:t,n.css("right",r*-1),n.css(u,r),n.css(e,r),f=o),n.css("width",f)},ItemClick:function(n){var t=this;n.preventDefault(),t.ClearHideTimer(),t.ClearShowTimer(),t.ClearPreventDwellTimer(),t._preventDwell=!0,t._preventClick==!1&&(t.StartPreventDwellTimer(),t.IsFlyoutVisible(n)?t.HideFlyouts():(t.HideFlyouts(),t.ShowFlyout(n),t.BiTrack(n,9)))},IsFlyoutVisible:function(n){return $(n.target).parents(".mstNavNavItem").find(".mstNavNavItemFlyout").css("display")!="none"},ShowFlyout:function(n){var f="selected",i=".mstNavNavItem",r=$(n.target).parents(i).addClass(f).find(".mstNavNavItemFlyout"),o=$(n.target).parents(i).position().top,e=$(i).outerHeight(),t,u;r.css("top",e+o-1),$(n.currentTarget).find(".mstNavNavItemText").addClass(f),t=$(n.target).parents(i).find(".mstNavNavFirstItem"),t.size()>0&&(u=$(n.target).parents(i).outerHeight(!0),document.documentElement.dir=="ltr"?t.css("left","-20px"):t.css("right","-20px"),t.css("height",u),t.css("display","block")),jQuery.browser.opera||jQuery.browser.msie&&document.documentMode<=7?r.show():r.slideDown(200)},HideFlyouts:function(){var t="selected",i=".mstNavNavItemFlyout";jQuery.browser.opera||jQuery.browser.msie&&document.documentMode<=7?$(i).hide():$(i).slideUp(200),$(".mstNavNavItem").removeClass(t),$(".mstNavNavItemText").removeClass(t),$(".mstNavNavFirstItem").css("display","none")},ItemDwell:function(n){var t=this;t.ClearPreventClickTimer(),t._preventClick=!0,t._preventDwell==!1&&(t.StartPreventClickTimer(),t.IsFlyoutVisible(n)?t.HideFlyouts():(t.HideFlyouts(),t.ShowFlyout(n)))},StartShowTimer:function(n){this._showTimerId=window.setTimeout($.proxy(function(){this.ItemDwell(n)},this),100)},StartHideTimer:function(){this._hideTimerId=window.setTimeout($.proxy(this.HideFlyouts,this),1e3)},StartPreventClickTimer:function(){this._preventClickTimerId=window.setTimeout($.proxy(function(){this._preventClick=!1},this),350)},StartPreventDwellTimer:function(){this._preventDwellTimerId=window.setTimeout($.proxy(function(){this._preventDwell=!1},this),350)},ClearShowTimer:function(){window.clearTimeout(this._showTimerId),this._showTimerId=null},ClearHideTimer:function(){window.clearTimeout(this._hideTimerId),this._hideTimerId=null},ClearPreventClickTimer:function(){window.clearTimeout(this._preventClickTimerId),this._preventClickTimerId=null},ClearPreventDwellTimer:function(){window.clearTimeout(this._preventDwellTimerId),this._preventDwellTimerId=null},BiTrack:function(n,t){return $.bi&&$.bi.dataRetrievers.structure&&(params={},params.interactiontype=t,params.cot=5,params.parenttypestructure=$.bi.dataRetrievers.structure.getTypeStructure($(n.target).parents(".mstNavNavItemTabText")),t==9&&(params.title=$(n.target).text()),$.bi.record(params)),!0}}