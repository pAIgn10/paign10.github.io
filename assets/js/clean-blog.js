/*!
 * Clean Blog v1.0.0 (http://startbootstrap.com)
 * Copyright 2015 Start Bootstrap
 * Licensed under MIT (https://spdx.org/licenses/MIT)
 */

// checks if element has requested class
function hasClass(element, cls) {
    return (' ' + element.className + ' ').indexOf(' ' + cls + ' ') > -1;
}

// returns viewport dimensions
// https://andylangton.co.uk/blog/development/get-viewportwindow-size-width-and-height-javascript
function viewport() {
    var e = window, a = 'inner';
    if ( !( 'innerWidth' in window ) ) {
        a = 'client';
        e = document.documentElement || document.body;
    }
    return { width: e[ a+'Width' ], height: e[ a+'Height' ] }
}

// tooltip init
$(function() {
    $("[data-toggle='tooltip']").tooltip();
});

// make images responsive
$(function() {
    $("img").addClass("img-responsive")
    // var imgs = document.getElementsByTagName('img');
    // for (var i = 0; i < imgs.length; i++) {
    //     if (!hasClass(imgs[i],'img-footer') && !hasClass(imgs[i],'img-portfolio'))
    //         imgs[i].className += " img-responsive";
    // }
});

// responsive tables
$(function() {
    $("table").wrap("<div class='table-responsive'></div>");
    $("table").addClass("table");
});

// responsive embed videos
$(function () {
    $('iframe[src*="youtube.com"]').wrap('<div class="embed-responsive embed-responsive-16by9"></div>');
    $('iframe[src*="youtube.com"]').addClass('embed-responsive-item');
    $('iframe[src*="vimeo.com"]').wrap('<div class="embed-responsive embed-responsive-16by9"></div>');
    $('iframe[src*="vimeo.com"]').addClass('embed-responsive-item');
});

// navigation scripts to show header on scroll-up
$(function($) {
    var MQL = 1170;

    //primary navigation slide-in effect
    if ($(window).width() > MQL) {
        var headerHeight = $('.navbar-custom').height();
        $(window).on('scroll', {
                previousTop: 0
            },
            function() {
                var currentTop = $(window).scrollTop();
                //check if user is scrolling up
                if (currentTop < this.previousTop) {
                    //if scrolling up...
                    if (currentTop > 0 && $('.navbar-custom').hasClass('is-fixed')) {
                        $('.navbar-custom').addClass('is-visible');
                    } else {
                        $('.navbar-custom').removeClass('is-visible is-fixed');
                    }
                } else {
                    //if scrolling down...
                    $('.navbar-custom').removeClass('is-visible');
                    if (currentTop > headerHeight && !$('.navbar-custom').hasClass('is-fixed')) $('.navbar-custom').addClass('is-fixed');
                }
                this.previousTop = currentTop;
            });
    }
});

// Portfolio

$(function() {
    if (window.location.href.toString().indexOf('portfolio') !== -1) {
        document.querySelector('html').setAttribute("style","height:100%");
        document.querySelector('body').setAttribute("style","min-height:101%");
    }
});

// MixItUp

var mixer;

$(function() {
    var containerEl = document.querySelector('[data-ref~="portfolio-container"]');
    if (containerEl == null) return;

    mixer = mixitup(containerEl, {
        animation: {
            effects: 'fade scale stagger(40ms)'
        },
        load: {
            filter: 'none'
        },
        selectors: {
            target: '[data-ref~="portfolio-target"]'
        }
    });

    containerEl.classList.add('portfolio-target-ready');

    mixer.show()
        .then(function() {
            mixer.configure({
                animation: {
                    effects: 'fade scale'
                }
            });
        });
});

function setPortfolioControlsClass() {
    if (viewport().width < 640) {
        $('#portfolio-controls').removeClass('btn-group');
        $('#portfolio-controls').addClass('btn-group-vertical');
    } else {
        $('#portfolio-controls').removeClass('btn-group-vertical');
        $('#portfolio-controls').addClass('btn-group');
    }
}

$(setPortfolioControlsClass());
window.onresize = setPortfolioControlsClass;

function sortProjects() {
    var el = document.querySelector('#portfolio-sort-btn');
    if (el == null) return;

    if (el.textContent === "Asc") {
        // el.setAttribute("data-sort","default:desc");
        mixer.sort('default:desc');
        el.textContent = "Desc";
    } else {
        // el.setAttribute("data-sort","default:asc");
        mixer.sort('default:Asc');
        el.textContent = "Asc";
    }
};
